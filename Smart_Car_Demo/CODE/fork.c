#include "fork.h"
#include "servo.h"
#include "buzzer.h"
#include "timer_pit.h"

//电机行进规律      ！！应当加入能够现场更改的按键程序，或在比赛现场更改，以适应2个三岔的情况
uint8 motor_pattern[4] = {0, 1, 0, 2}; //直 右横 直 左横  （前行）

//舵机/旋转规律
uint8 circle_pattern[4] = {0, 1, 2, 3}; //舵机：中 左 中 右   电机：右 右 左 左

//电机/舵机/旋转规律 控制选择号p
uint8 p;

//icm20602采集和车身旋转标志位
uint8 icm_flag;

//三岔信号量
rt_sem_t fork_sem;

void fork_entry(void *parameter)
{
    while(1)
    {
        //接收三岔的信号量
        rt_sem_take(fork_sem, RT_WAITING_FOREVER);
        //rt_mb_send(buzzer_mailbox, 400);//蜂鸣器发出声音,提醒三岔识别位置

        //出入三岔时刻,进行下述操作

        p++;
        if(p > 3)
        {
            p = 0;

        }

        icm_flag = 1; //使能车身原地旋转，且icm20602进行采集

        rt_sem_release(servo_sem); //控制舵机旋转

        //rt_mb_send(buzzer_mailbox, 400);//蜂鸣器发出声音,提醒三岔识别位置
        rt_thread_mdelay(500);
    }
}

void fork_init(void)
{
    rt_thread_t tid2;

    fork_sem = rt_sem_create("fork", 0, RT_IPC_FLAG_FIFO);

    tid2 = rt_thread_create("fork", fork_entry, RT_NULL, 512, 4, 2);

    if(RT_NULL != tid2)
    {
        rt_thread_startup(tid2);
    }
}
