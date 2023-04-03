#include "fork.h"
#include "servo.h"
#include "buzzer.h"
#include "timer_pit.h"

//����н�����      ����Ӧ�������ܹ��ֳ����ĵİ������򣬻��ڱ����ֳ����ģ�����Ӧ2����������
uint8 motor_pattern[4] = {0, 1, 0, 2}; //ֱ �Һ� ֱ ���  ��ǰ�У�

//���/��ת����
uint8 circle_pattern[4] = {0, 1, 2, 3}; //������� �� �� ��   ������� �� �� ��

//���/���/��ת���� ����ѡ���p
uint8 p;

//icm20602�ɼ��ͳ�����ת��־λ
uint8 icm_flag;

//�����ź���
rt_sem_t fork_sem;

void fork_entry(void *parameter)
{
    while(1)
    {
        //����������ź���
        rt_sem_take(fork_sem, RT_WAITING_FOREVER);
        //rt_mb_send(buzzer_mailbox, 400);//��������������,��������ʶ��λ��

        //��������ʱ��,������������

        p++;
        if(p > 3)
        {
            p = 0;

        }

        icm_flag = 1; //ʹ�ܳ���ԭ����ת����icm20602���вɼ�

        rt_sem_release(servo_sem); //���ƶ����ת

        //rt_mb_send(buzzer_mailbox, 400);//��������������,��������ʶ��λ��
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
