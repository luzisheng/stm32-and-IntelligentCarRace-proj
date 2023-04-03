/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//整套推荐IO查看Projecct文件夹下的TXT文本

//打开新的工程或者工程移动了位置务必执行以下操作
//右键单击工程，选择刷新

#include "headfile.h"
#include "display.h"
#include "camera_process.h"
#include "timer_pit.h"
#include "encoder.h"
#include "buzzer.h"
#include "button.h"
#include "motor.h"
#include "fork.h"
#include "servo.h"
#include "elec.h"

rt_sem_t camera_sem;


int main(void)
{
    uint8  start_flag = 1; //出库标志
    uint32 start_count = 0;//出库延时计数

    camera_sem = rt_sem_create("camera", 0, RT_IPC_FLAG_FIFO);
    mt9v03x_init();  //采集完成释放信号量
    icm20602_init_spi();
    //display_init();  //     线程 优先级6
    Encoder_Init();
    buzzer_init();   //邮箱   线程 优先级5
    button_init();   //信号量 软件定时
    servo_init();    //信号量 线程 优先级5
    fork_init();     //事件   线程 优先级5
    Motor_Init();
    //elec_init();
    timer_pit_init();



    //开环出库左转弯
    while(start_flag)
    {
        set_speed = 300;
        kp = 58;
        turn_speed = -121;
        Motor_PID();
        Motor_Control(out[0], out[1], out[2], out[3]);
        start_count++;
        if(start_count > 70000)
        {
            start_flag = 0;//退出开环出库，进入寻迹
        }
    }

    /*参数设置*/
    /*差速pid未加入积分项，目前只用了pd算法*/
    /*目前低速时set_speed给200即可，但是kp不可以比15小，否则静差过大   速度太慢*/
    set_speed = 300;     //测试给200和40，但事实300和60稳跑！！！
   // KP = 60;
   // KD = 0;
    kp = 58;

    kp_heng = 43;


    set_speed_heng = 290;
    set_speed_ratio = 1;//set_speed_heng/set_speed 即 100/200

    while(1)
    {
        if(!stop_flag)
        {
            //等待摄像头采集完毕
            rt_sem_take(camera_sem, RT_WAITING_FOREVER);

            //开始处理摄像头图像
            if(!icm_flag)
            {
                getborder(&mt9v03x_image[0]);
                turn_speed = motor_turn(midLine);
            }

            //释放CPU控制权
            rt_thread_mdelay(10);
        }
        //停车（不再处理新图像，故放在释放cpu控制权后）  转弯角度在camera_process中
        else if(stop_flag == 1)
        {
            rt_thread_mdelay(220); //开环转弯延时时间
            turn_speed = -180;
            rt_thread_mdelay(360); //开环转弯延时时间
            stop_flag = 2;
        }
        else if(stop_flag == 2)
        {
            set_speed = 0;
            set_speed_heng = 0;
            turn_speed = 0;
            Motor_Control(0, 0, 0, 0);
        }
        //变速！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        if(circle_flag2 == 2)
        {
            set_speed = 380;
        }

        //去除注释下面要加else
        else if (turn_speed > 60 || turn_speed < -60)
        {
            set_speed = 300;
        }
        else
        {
            set_speed = 300;
        }
    }
}
