#include "encoder.h"
#include "motor.h"
#include "buzzer.h"
#include "timer_pit.h"
#include "camera_process.h"
#include "fork.h"
#include "elec.h"

float integral_gyro_z = 0;//z���������

void timer1_pit_entry(void *parameter)
{
    static uint32 time;
    time++;

    //�ɼ�����������
    //��pwm����1000ʱencoder_data��15������
    encoder_get();
    
    //��������ٶȻ�
    Motor_PID();
    
    //�ɼ�����������
    if(icm_flag)            //��Ӧ����ʱ
    {
        get_icm20602_gyro_spi();
        integral_gyro_z += (float)(icm_gyro_z+3)*0.006;

        switch(p){   //��׼ֵ��90���Ӧ4000
        case 0:
        case 2:     //����ת150��-600
            if(integral_gyro_z > 6066 || integral_gyro_z < -6066)
            {
                integral_gyro_z = 0;
                icm_flag = 0; //�˳�icm�ɼ��ͳ���ԭ����ת
            }
            break;
        case 1:
        case 3:     //����ת30��
            if(integral_gyro_z > 333 || integral_gyro_z < -333)
            {
                integral_gyro_z = 0;
                icm_flag = 0; //�˳�icm�ɼ��ͳ���ԭ����ת
            }
            break;
        default:
            break;
        }
    }

    if(circle_flag2 == 1)  //��Ӧ����ʱ��circle_flag2 Ϊ1��2������������ǻ��֣�Ϊ3׼��������
    {
        get_icm20602_gyro_spi();
        integral_gyro_z += (float)(icm_gyro_z+3)*0.006;

        if(integral_gyro_z > 888 || integral_gyro_z < -888) //��������������ֱ��������ת��20��
        {
            //integral_gyro_z = 0;
            circle_flag2 = 2; //�˳�����������������ջ�Ѱ����������ʱ�ٴ�ʹ��
        }
    }

    if(circle_flag2 == 2)  //��Ӧ����ʱ��circle_flag2 Ϊ1��2������������ǻ��֣�Ϊ3׼��������
    {
        get_icm20602_gyro_spi();
        integral_gyro_z += (float)(icm_gyro_z+3)*0.006;

        if(integral_gyro_z > 12000 || integral_gyro_z < -12000) //�����׼������������ת��270�ȣ�׼��������
        {
            integral_gyro_z = 0;
            circle_flag2 = 3; //�˳������ջ���׼������������
            //rt_mb_send(buzzer_mailbox, 400);//��������������,��������ʶ��λ��
        }
    }

    if(circle_flag2 == 3)  //��Ӧ����ʱ��circle_flag2 Ϊ1��2������������ǻ��֣�Ϊ3׼��������
    {
        get_icm20602_gyro_spi();
        integral_gyro_z += (float)(icm_gyro_z+3)*0.006;

        if(integral_gyro_z > 3700 || integral_gyro_z < -3700) //��ת60���㣬
        {
            integral_gyro_z = 0;
            circle_flag2 = 4; //�˳������ջ���׼������������
        }
    }

    
    //�ɼ�����ź�
//    elec_get();
    
    //���ݵ���źż��㳵��λ��
//    elec_calculate();
    
    
    //��ͼ���������ĳ���λ�������źż�����ĳ���λ�ý����ںϵõ���������
    //Ȼ����г�ģ����
    

    
    //���Ƶ��ת��
    Motor_Control(out[0], out[1], out[2], out[3]);
   // Motor_Control(3600, 2400, 3000, 3000);
    //Motor_Control(1500, 1200, 900, 500);
}


void timer_pit_init(void)
{
    rt_timer_t timer;
    
    //����һ����ʱ�� ��������
    timer = rt_timer_create("timer1", timer1_pit_entry, RT_NULL, 2, RT_TIMER_FLAG_PERIODIC);
    
    //������ʱ��
    if(RT_NULL != timer)
    {
        rt_timer_start(timer);
    }

    
}
