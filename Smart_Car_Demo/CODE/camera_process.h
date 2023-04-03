#ifndef __CAMERA_PROCESS_H
#define __CAMERA_PROCESS_H

#include "common.h"
#include "SEEKFREE_MT9V03X.h"


#define THRESHOLD   105   //��ֵ����ֵ
#define CAMERA_KP   0.32   //Ѳ��ʱ����turn_speed�ı���ϵ��

extern uint8 leftBorder[MT9V03X_H];
extern uint8 rightBorder[MT9V03X_H];
extern uint8 midLine[MT9V03X_H];

extern uint8 circle_flag1;  //����ͻȻ���߱�־
extern uint8 circle_flag2;  //������ʾ��������������

extern uint8 garage_get_flag; //Ѱ�������־
extern uint8 stop_flag; //ͣ����־


void getborder(uint8 (*img)[MT9V03X_W]);//�߽紦��,Ѱ������,����Ϊʡ���еĶ�ά���飬������ͷ����mt9v03x_image[MT9V03X_H][MT9V03X_W];
short int motor_turn(uint8 mid[MT9V03X_H]); //�������߿���ƫ���ٶ�turn_speed���������100�޷�����

#endif
