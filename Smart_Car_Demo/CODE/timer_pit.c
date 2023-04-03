#include "encoder.h"
#include "motor.h"
#include "buzzer.h"
#include "timer_pit.h"
#include "camera_process.h"
#include "fork.h"
#include "elec.h"

float integral_gyro_z = 0;//z轴积分数据

void timer1_pit_entry(void *parameter)
{
    static uint32 time;
    time++;

    //采集编码器数据
    //在pwm给定1000时encoder_data≈15？？？
    encoder_get();
    
    //电机控制速度环
    Motor_PID();
    
    //采集陀螺仪数据
    if(icm_flag)            //对应三岔时
    {
        get_icm20602_gyro_spi();
        integral_gyro_z += (float)(icm_gyro_z+3)*0.006;

        switch(p){   //基准值：90°对应4000
        case 0:
        case 2:     //左右转150°-600
            if(integral_gyro_z > 6066 || integral_gyro_z < -6066)
            {
                integral_gyro_z = 0;
                icm_flag = 0; //退出icm采集和车身原地旋转
            }
            break;
        case 1:
        case 3:     //左右转30°
            if(integral_gyro_z > 333 || integral_gyro_z < -333)
            {
                integral_gyro_z = 0;
                icm_flag = 0; //退出icm采集和车身原地旋转
            }
            break;
        default:
            break;
        }
    }

    if(circle_flag2 == 1)  //对应环岛时，circle_flag2 为1或2都会进入陀螺仪积分，为3准备出环岛
    {
        get_icm20602_gyro_spi();
        integral_gyro_z += (float)(icm_gyro_z+3)*0.006;

        if(integral_gyro_z > 888 || integral_gyro_z < -888) //开环进出环岛，直到车身旋转满20度
        {
            //integral_gyro_z = 0;
            circle_flag2 = 2; //退出开环进环岛，进入闭环寻迹，出环岛时再次使用
        }
    }

    if(circle_flag2 == 2)  //对应环岛时，circle_flag2 为1或2都会进入陀螺仪积分，为3准备出环岛
    {
        get_icm20602_gyro_spi();
        integral_gyro_z += (float)(icm_gyro_z+3)*0.006;

        if(integral_gyro_z > 12000 || integral_gyro_z < -12000) //车身从准备进环岛到旋转了270度，准备出环岛
        {
            integral_gyro_z = 0;
            circle_flag2 = 3; //退出环岛闭环，准备开环出环岛
            //rt_mb_send(buzzer_mailbox, 400);//蜂鸣器发出声音,提醒三岔识别位置
        }
    }

    if(circle_flag2 == 3)  //对应环岛时，circle_flag2 为1或2都会进入陀螺仪积分，为3准备出环岛
    {
        get_icm20602_gyro_spi();
        integral_gyro_z += (float)(icm_gyro_z+3)*0.006;

        if(integral_gyro_z > 3700 || integral_gyro_z < -3700) //旋转60多点°，
        {
            integral_gyro_z = 0;
            circle_flag2 = 4; //退出环岛闭环，准备开环出环岛
        }
    }

    
    //采集电磁信号
//    elec_get();
    
    //根据电磁信号计算车身位置
//    elec_calculate();
    
    
    //将图像计算出来的车身位置与电磁信号计算出的车身位置进行融合得到更好数据
    //然后进行车模控制
    

    
    //控制电机转动
    Motor_Control(out[0], out[1], out[2], out[3]);
   // Motor_Control(3600, 2400, 3000, 3000);
    //Motor_Control(1500, 1200, 900, 500);
}


void timer_pit_init(void)
{
    rt_timer_t timer;
    
    //创建一个定时器 周期运行
    timer = rt_timer_create("timer1", timer1_pit_entry, RT_NULL, 2, RT_TIMER_FLAG_PERIODIC);
    
    //启动定时器
    if(RT_NULL != timer)
    {
        rt_timer_start(timer);
    }

    
}
