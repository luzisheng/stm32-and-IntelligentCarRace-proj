#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "headfile.h"

#define MOTOR1_A   PWM2_CH3_B10   //定义1电机正转PWM引脚
#define MOTOR1_B   PWM4_CH3_B8    //定义1电机反转PWM引脚

#define MOTOR2_A   PWM2_CH4_B11   //定义2电机正转PWM引脚
#define MOTOR2_B   PWM4_CH4_B9    //定义2电机反转PWM引脚

#define MOTOR3_A   PWM2_CH1_A15   //定义3电机正转PWM引脚
#define MOTOR3_B   PWM4_CH1_B6    //定义3电机反转PWM引脚

#define MOTOR4_A   PWM2_CH2_B3    //定义4电机正转PWM引脚
#define MOTOR4_B   PWM4_CH2_B7    //定义4电机反转PWM引脚

extern int32 speed1_power;      //电机1占空比，通过PID改变数值
extern int32 speed2_power;      //电机2占空比，通过PID改变数值
extern int32 speed3_power;      //电机3占空比，通过PID改变数值
extern int32 speed4_power;      //电机4占空比，通过PID改变数值

extern int16 set_speed;  //期望速度，相对速度，设定值取-511~511之间
extern int16 turn_speed; //转向速度，相对速度，设定值取-511~511之间

extern int16 set_speed_heng;//恒跑
extern float set_speed_ratio; //横跑比例
extern int16 turn_speed; //转向速度，相对速度，设定值取0~4095之间
extern uint32 kp_heng;

extern uint32 kp;   //PID比例系数
extern uint32 ki;   //PID积分系数
extern uint32 kd;   //PID微分系数
extern int32 out[4];          //输出量
extern int16 ek[4], ek1[4]; //前后误差
extern uint32 KP; //PID比例系数
extern uint32 KD; //PID积分系数
extern int16 error,last_error;

void Motor_Init(void);       //初始化电机PWM引脚和方向引脚函数
void Motor_PID(void);
void Motor_Control(int32 speed1_power, int32 speed2_power, int32 speed3_power, int32 speed4_power);//计算电机转向和PWM占空比,控制速度输出

#endif

