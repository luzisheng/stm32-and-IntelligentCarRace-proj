#ifndef __CAMERA_PROCESS_H
#define __CAMERA_PROCESS_H

#include "common.h"
#include "SEEKFREE_MT9V03X.h"


#define THRESHOLD   105   //二值化阈值
#define CAMERA_KP   0.32   //巡线时控制turn_speed的比例系数

extern uint8 leftBorder[MT9V03X_H];
extern uint8 rightBorder[MT9V03X_H];
extern uint8 midLine[MT9V03X_H];

extern uint8 circle_flag1;  //环岛突然丢线标志
extern uint8 circle_flag2;  //环岛显示并且驱动蜂鸣器

extern uint8 garage_get_flag; //寻到车库标志
extern uint8 stop_flag; //停车标志


void getborder(uint8 (*img)[MT9V03X_W]);//边界处理,寻找中线,传参为省略行的二维数组，即摄像头数据mt9v03x_image[MT9V03X_H][MT9V03X_W];
short int motor_turn(uint8 mid[MT9V03X_H]); //根据中线控制偏差速度turn_speed输出，已做100限幅处理

#endif
