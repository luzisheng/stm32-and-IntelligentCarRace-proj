#ifndef __ENCODER_H
#define __ENCODER_H

#include "common.h"

extern uint16 encoder_data[4]; //³µËÙ£¬-511 ~ +511

void Encoder_Init(void);
void encoder_get(void);

#endif
