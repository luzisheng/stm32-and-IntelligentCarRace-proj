#include "servo.h"
#include "fork.h"
#include "button.h"
#include "buzzer.h"

//定义舵机引脚 由于电机驱动多使用了定时器2，所以舵机给定时器3
#define  PWM2_CH  PWM3_CH1_A6

//舵机信号量
rt_sem_t servo_sem;

void servo_entry(void *parameter)
{
    while(1)
    {
        //接收判断三岔之后的信号量
        rt_sem_take(servo_sem, RT_WAITING_FOREVER);
        //选择舵机旋转方式
        switch(circle_pattern[p]){
        case 0:
            pwm_duty(PWM2_CH, 750);  //舵机回中间
            break;
        case 1:
            pwm_duty(PWM2_CH, 1250);  //舵机左转
            break;
        case 2:
            pwm_duty(PWM2_CH, 750);  //舵机回中间
            break;
        case 3:
            pwm_duty(PWM2_CH, 245); //舵机右转
            break;
        default:
            break;
        }
    }
}

void servo_init(void)
{
    rt_thread_t tid;

    pwm_init(PWM2_CH, 50, 750); //初始化舵机和定时器3

    servo_sem = rt_sem_create("servo", 0, RT_IPC_FLAG_FIFO);

    tid = rt_thread_create("servo", servo_entry, RT_NULL, 512, 5, 2);

    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
