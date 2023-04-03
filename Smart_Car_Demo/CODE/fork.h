#ifndef _fork_h
#define _fork_h

#include "headfile.h"

//�����־λ
#define EVENT_FLAG3 (1<<3)  //ʶ��Ϊ˫�߲����ߵ���ͨ����
#define EVENT_FLAG5 (1<<5)  //ʶ��Ϊ˫�߶��ߣ�������ǰ�����ֺڵ������

extern rt_sem_t fork_sem;
extern uint8 motor_pattern[4];   //����н�����
extern uint8 circle_pattern[4];  //���/��ת����
extern uint8 p;                  //���/���/��ת���� ����ѡ���p
extern uint8 icm_flag;           //icm20602�ɼ��ͳ�����ת��־λ

void fork_init(void);

#endif
