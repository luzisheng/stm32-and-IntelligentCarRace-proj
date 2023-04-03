#include "servo.h"
#include "fork.h"
#include "button.h"
#include "buzzer.h"

//���������� ���ڵ��������ʹ���˶�ʱ��2�����Զ������ʱ��3
#define  PWM2_CH  PWM3_CH1_A6

//����ź���
rt_sem_t servo_sem;

void servo_entry(void *parameter)
{
    while(1)
    {
        //�����ж�����֮����ź���
        rt_sem_take(servo_sem, RT_WAITING_FOREVER);
        //ѡ������ת��ʽ
        switch(circle_pattern[p]){
        case 0:
            pwm_duty(PWM2_CH, 750);  //������м�
            break;
        case 1:
            pwm_duty(PWM2_CH, 1250);  //�����ת
            break;
        case 2:
            pwm_duty(PWM2_CH, 750);  //������м�
            break;
        case 3:
            pwm_duty(PWM2_CH, 245); //�����ת
            break;
        default:
            break;
        }
    }
}

void servo_init(void)
{
    rt_thread_t tid;

    pwm_init(PWM2_CH, 50, 750); //��ʼ������Ͷ�ʱ��3

    servo_sem = rt_sem_create("servo", 0, RT_IPC_FLAG_FIFO);

    tid = rt_thread_create("servo", servo_entry, RT_NULL, 512, 5, 2);

    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
