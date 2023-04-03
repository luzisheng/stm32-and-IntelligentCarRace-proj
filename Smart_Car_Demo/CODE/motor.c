#include "motor.h"
#include "fork.h"
#include "camera_process.h"
#include "encoder.h"

int32 speed1_power;      //电机1占空比
int32 speed2_power;      //电机2占空比
int32 speed3_power;      //电机3占空比
int32 speed4_power;      //电机4占空比

int16 set_speed;//期望速度，相对速度，设定值取0~4095之间
int16 set_speed_heng;//恒跑
float set_speed_ratio; //横跑比例
int16 turn_speed; //转向速度，相对速度，设定值取0~4095之间
uint32 kp_heng;
uint32 kp; //PID比例系数
uint32 ki; //PID积分系数
uint32 kd; //PID微分系数

uint32 KP; //PID比例系数
uint32 KD; //PID积分系数
int16 error,last_error;


int32 out[4];          //输出量
int16 ek[4], ek1[4]; //前后次误差

//-------------------------------------------------------------------------------------------------------------------
//  @brief          初始化电机PWM引脚和方向引脚
//  @parameter      void
//  @return         void
//  Sample usage:   Motor_Init();
//-------------------------------------------------------------------------------------------------------------------
void Motor_Init(void)
{
    pwm_init(MOTOR1_A,17000,0);
    pwm_init(MOTOR1_B,17000,0);
    pwm_init(MOTOR2_A,17000,0);
    pwm_init(MOTOR2_B,17000,0);
    pwm_init(MOTOR3_A,17000,0);
    pwm_init(MOTOR3_B,17000,0);
    pwm_init(MOTOR4_A,17000,0);
    pwm_init(MOTOR4_B,17000,0);

    //设置默认初始速度  也可以通过在线调试直接修改此值  变化电机速度
    speed1_power = 1000;
    speed2_power = 1000;
    speed3_power = 1000;
    speed4_power = 1000;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief          电机PID控制
//  @parameter      void
//  @return         电机占空比
//  Sample usage:   Motor_PID(void)
//-------------------------------------------------------------------------------------------------------------------
void Motor_PID(void)
{/*
    error = (encoder_data[0]+encoder_data[2]-encoder_data[1]-encoder_data[3])/4 - turn_speed;
    last_error = error;
    turn_speed = KP*error+KD*(error - last_error);

    out[0] = (int32)( set_speed + turn_speed);
    out[1] = (int32)( set_speed + turn_speed);
    out[2] = (int32)( set_speed + turn_speed);
    out[3] = (int32)( set_speed + turn_speed);
*/
  ek1[0] = ek[0]; //保存上次误差
  ek1[1] = ek[1];
  ek1[2] = ek[2];
  ek1[3] = ek[3];

  //前行时
  if(!icm_flag)
  {
      switch(motor_pattern[p]){
      case 0:
          //直行
          ek[0]  = (set_speed + turn_speed) - encoder_data[0];//左边上轮
          ek[2]  = (set_speed + turn_speed) - encoder_data[2];//左边下轮
          ek[1]  = (set_speed - turn_speed) - encoder_data[1];//右边上轮
          ek[3]  = (set_speed - turn_speed) - encoder_data[3];//右边下轮
          break;
      case 1:
          //车身右转后接横行
          ek[0]  = (-set_speed_heng + set_speed_ratio*turn_speed) - encoder_data[0];//左边上轮
          ek[2]  = ( set_speed_heng + set_speed_ratio*turn_speed) - encoder_data[2];//左边下轮
          ek[1]  = ( set_speed_heng - set_speed_ratio*turn_speed) - encoder_data[1];//右边上轮
          ek[3]  = (-set_speed_heng - set_speed_ratio*turn_speed) - encoder_data[3];//右边下轮
          break;
      case 2:
          //车身左转后接横行
          ek[0]  = ( set_speed_heng + set_speed_ratio*turn_speed) - encoder_data[0];//左边上轮
          ek[2]  = (-set_speed_heng + set_speed_ratio*turn_speed) - encoder_data[2];//左边下轮
          ek[1]  = (-set_speed_heng - set_speed_ratio*turn_speed) - encoder_data[1];//右边上轮
          ek[3]  = ( set_speed_heng - set_speed_ratio*turn_speed) - encoder_data[3];//右边下轮
          break;
      default:
          break;
      }
  }
  //旋转时
  else{
      switch(circle_pattern[p]){
      case 0:
          //车身右转
          ek[0]  =  250 - encoder_data[0];
          ek[2]  =  250 - encoder_data[2];
          ek[1]  = -250 - encoder_data[1];
          ek[3]  = -250 - encoder_data[3];
          break;
      case 1:
          //车身右转
          ek[0]  =  250 - encoder_data[0];
          ek[2]  =  250 - encoder_data[2];
          ek[1]  = -250 - encoder_data[1];
          ek[3]  = -250 - encoder_data[3];
          break;
      case 2:
          //车身左转
          ek[0]  = -250 - encoder_data[0];
          ek[2]  = -250 - encoder_data[2];
          ek[1]  =  250 - encoder_data[1];
          ek[3]  =  250 - encoder_data[3];
          break;
      case 3:
          //车身左转
          ek[0]  = -250 - encoder_data[0];
          ek[2]  = -250 - encoder_data[2];
          ek[1]  =  250 - encoder_data[1];
          ek[3]  =  250 - encoder_data[3];
          break;
      default:
          break;
      }
  }

  if(!motor_pattern[p])
  {
      //使用函数前先设置PID参数
      out[0] = (int32)( kp*ek[0] + kd*(ek[0] - ek1[0]) );
      out[1] = (int32)( kp*ek[1] + kd*(ek[1] - ek1[1]) );
      out[2] = (int32)( kp*ek[2] + kd*(ek[2] - ek1[2]) );
      out[3] = (int32)( kp*ek[3] + kd*(ek[3] - ek1[3]) );
  }
  else {
      out[0] = (int32)( kp_heng*ek[0] );
      out[1] = (int32)( kp_heng*ek[1] );
      out[2] = (int32)( kp_heng*ek[2] );
      out[3] = (int32)( kp_heng*ek[3] );
  }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief          转速输出控制，由speedx_power信号计算电机转向和PWM占空比
//  @parameter      speedx_power，由增量式PID改变数值
//  @return         void
//  Sample usage:   Motor_Control(1000, 1000, 1000, 1000);4个电机正转 设置占空比为 百分之 (speedx_power即1000/TIMER1_PWM_DUTY_MAX*100)
                                                         //其中最大占空比值PWM_DUTY_MAX 可以在fsl_pwm.h文件中修改 默认为10000
//-------------------------------------------------------------------------------------------------------------------
void Motor_Control(int32 speed1_power, int32 speed2_power, int32 speed3_power, int32 speed4_power)
{
    if(0<=speed1_power) //电机1   正转 设置占空比为 百分之 (1000/TIMER1_PWM_DUTY_MAX*100)
    {
        pwm_duty(MOTOR1_A, speed1_power);
        pwm_duty(MOTOR1_B, 0);
    }
    else                //电机1   反转
    {
        pwm_duty(MOTOR1_A, 0);
        pwm_duty(MOTOR1_B, -speed1_power);
    }

    if(0<=speed2_power) //电机2   正转
    {
        pwm_duty(MOTOR2_A, speed2_power);
        pwm_duty(MOTOR2_B, 0);
    }
    else                //电机2   反转
    {
        pwm_duty(MOTOR2_A, 0);
        pwm_duty(MOTOR2_B, -speed2_power);
    }

    if(0<=speed3_power) //电机3   正转
    {
        pwm_duty(MOTOR3_A, speed3_power);
        pwm_duty(MOTOR3_B, 0);
    }
    else                //电机3   反转
    {
        pwm_duty(MOTOR3_A, 0);
        pwm_duty(MOTOR3_B, -speed3_power);
    }

    if(0<=speed4_power) //电机3   正转
    {
        pwm_duty(MOTOR4_A, speed4_power);
        pwm_duty(MOTOR4_B, 0);
    }
    else                //电机3   反转
    {
        pwm_duty(MOTOR4_A, 0);
        pwm_duty(MOTOR4_B, -speed4_power);
    }
}
