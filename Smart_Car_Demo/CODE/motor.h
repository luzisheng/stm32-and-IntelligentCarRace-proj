#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "headfile.h"

#define MOTOR1_A   PWM2_CH3_B10   //����1�����תPWM����
#define MOTOR1_B   PWM4_CH3_B8    //����1�����תPWM����

#define MOTOR2_A   PWM2_CH4_B11   //����2�����תPWM����
#define MOTOR2_B   PWM4_CH4_B9    //����2�����תPWM����

#define MOTOR3_A   PWM2_CH1_A15   //����3�����תPWM����
#define MOTOR3_B   PWM4_CH1_B6    //����3�����תPWM����

#define MOTOR4_A   PWM2_CH2_B3    //����4�����תPWM����
#define MOTOR4_B   PWM4_CH2_B7    //����4�����תPWM����

extern int32 speed1_power;      //���1ռ�ձȣ�ͨ��PID�ı���ֵ
extern int32 speed2_power;      //���2ռ�ձȣ�ͨ��PID�ı���ֵ
extern int32 speed3_power;      //���3ռ�ձȣ�ͨ��PID�ı���ֵ
extern int32 speed4_power;      //���4ռ�ձȣ�ͨ��PID�ı���ֵ

extern int16 set_speed;  //�����ٶȣ�����ٶȣ��趨ֵȡ-511~511֮��
extern int16 turn_speed; //ת���ٶȣ�����ٶȣ��趨ֵȡ-511~511֮��

extern int16 set_speed_heng;//����
extern float set_speed_ratio; //���ܱ���
extern int16 turn_speed; //ת���ٶȣ�����ٶȣ��趨ֵȡ0~4095֮��
extern uint32 kp_heng;

extern uint32 kp;   //PID����ϵ��
extern uint32 ki;   //PID����ϵ��
extern uint32 kd;   //PID΢��ϵ��
extern int32 out[4];          //�����
extern int16 ek[4], ek1[4]; //ǰ�����
extern uint32 KP; //PID����ϵ��
extern uint32 KD; //PID����ϵ��
extern int16 error,last_error;

void Motor_Init(void);       //��ʼ�����PWM���źͷ������ź���
void Motor_PID(void);
void Motor_Control(int32 speed1_power, int32 speed2_power, int32 speed3_power, int32 speed4_power);//������ת���PWMռ�ձ�,�����ٶ����

#endif

