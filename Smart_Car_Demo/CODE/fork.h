#ifndef _fork_h
#define _fork_h

#include "headfile.h"

//三岔标志位
#define EVENT_FLAG3 (1<<3)  //识别为双边不丢线的普通赛道
#define EVENT_FLAG5 (1<<5)  //识别为双边丢线，正中线前方出现黑点的三岔

extern rt_sem_t fork_sem;
extern uint8 motor_pattern[4];   //电机行进规律
extern uint8 circle_pattern[4];  //舵机/旋转规律
extern uint8 p;                  //电机/舵机/旋转规律 控制选择号p
extern uint8 icm_flag;           //icm20602采集和车身旋转标志位

void fork_init(void);

#endif
