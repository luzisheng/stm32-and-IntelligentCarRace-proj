/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            main
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ790875685)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�

//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//�Ҽ��������̣�ѡ��ˢ��

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
    uint8  start_flag = 1; //�����־
    uint32 start_count = 0;//������ʱ����

    camera_sem = rt_sem_create("camera", 0, RT_IPC_FLAG_FIFO);
    mt9v03x_init();  //�ɼ�����ͷ��ź���
    icm20602_init_spi();
    //display_init();  //     �߳� ���ȼ�6
    Encoder_Init();
    buzzer_init();   //����   �߳� ���ȼ�5
    button_init();   //�ź��� �����ʱ
    servo_init();    //�ź��� �߳� ���ȼ�5
    fork_init();     //�¼�   �߳� ���ȼ�5
    Motor_Init();
    //elec_init();
    timer_pit_init();



    //����������ת��
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
            start_flag = 0;//�˳��������⣬����Ѱ��
        }
    }

    /*��������*/
    /*����pidδ��������Ŀǰֻ����pd�㷨*/
    /*Ŀǰ����ʱset_speed��200���ɣ�����kp�����Ա�15С�����򾲲����   �ٶ�̫��*/
    set_speed = 300;     //���Ը�200��40������ʵ300��60���ܣ�����
   // KP = 60;
   // KD = 0;
    kp = 58;

    kp_heng = 43;


    set_speed_heng = 290;
    set_speed_ratio = 1;//set_speed_heng/set_speed �� 100/200

    while(1)
    {
        if(!stop_flag)
        {
            //�ȴ�����ͷ�ɼ����
            rt_sem_take(camera_sem, RT_WAITING_FOREVER);

            //��ʼ��������ͷͼ��
            if(!icm_flag)
            {
                getborder(&mt9v03x_image[0]);
                turn_speed = motor_turn(midLine);
            }

            //�ͷ�CPU����Ȩ
            rt_thread_mdelay(10);
        }
        //ͣ�������ٴ�����ͼ�񣬹ʷ����ͷ�cpu����Ȩ��  ת��Ƕ���camera_process��
        else if(stop_flag == 1)
        {
            rt_thread_mdelay(220); //����ת����ʱʱ��
            turn_speed = -180;
            rt_thread_mdelay(360); //����ת����ʱʱ��
            stop_flag = 2;
        }
        else if(stop_flag == 2)
        {
            set_speed = 0;
            set_speed_heng = 0;
            turn_speed = 0;
            Motor_Control(0, 0, 0, 0);
        }
        //���٣�����������������������������������������������������������������������������������������
        if(circle_flag2 == 2)
        {
            set_speed = 380;
        }

        //ȥ��ע������Ҫ��else
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
