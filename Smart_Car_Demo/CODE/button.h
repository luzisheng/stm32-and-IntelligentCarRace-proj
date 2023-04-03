#ifndef _button_h
#define _button_h

#include "headfile.h"

//开关信号量
extern rt_sem_t key1_sem;
extern rt_sem_t key2_sem;
extern rt_sem_t key3_sem;

void button_init(void);

#endif
