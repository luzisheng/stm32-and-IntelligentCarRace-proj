#include "motor.h"
#include "fork.h"
#include "camera_process.h"
#include "encoder.h"

int32 speed1_power;      //���1ռ�ձ�
int32 speed2_power;      //���2ռ�ձ�
int32 speed3_power;      //���3ռ�ձ�
int32 speed4_power;      //���4ռ�ձ�

int16 set_speed;//�����ٶȣ�����ٶȣ��趨ֵȡ0~4095֮��
int16 set_speed_heng;//����
float set_speed_ratio; //���ܱ���
int16 turn_speed; //ת���ٶȣ�����ٶȣ��趨ֵȡ0~4095֮��
uint32 kp_heng;
uint32 kp; //PID����ϵ��
uint32 ki; //PID����ϵ��
uint32 kd; //PID΢��ϵ��

uint32 KP; //PID����ϵ��
uint32 KD; //PID����ϵ��
int16 error,last_error;


int32 out[4];          //�����
int16 ek[4], ek1[4]; //ǰ������

//-------------------------------------------------------------------------------------------------------------------
//  @brief          ��ʼ�����PWM���źͷ�������
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

    //����Ĭ�ϳ�ʼ�ٶ�  Ҳ����ͨ�����ߵ���ֱ���޸Ĵ�ֵ  �仯����ٶ�
    speed1_power = 1000;
    speed2_power = 1000;
    speed3_power = 1000;
    speed4_power = 1000;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief          ���PID����
//  @parameter      void
//  @return         ���ռ�ձ�
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
  ek1[0] = ek[0]; //�����ϴ����
  ek1[1] = ek[1];
  ek1[2] = ek[2];
  ek1[3] = ek[3];

  //ǰ��ʱ
  if(!icm_flag)
  {
      switch(motor_pattern[p]){
      case 0:
          //ֱ��
          ek[0]  = (set_speed + turn_speed) - encoder_data[0];//�������
          ek[2]  = (set_speed + turn_speed) - encoder_data[2];//�������
          ek[1]  = (set_speed - turn_speed) - encoder_data[1];//�ұ�����
          ek[3]  = (set_speed - turn_speed) - encoder_data[3];//�ұ�����
          break;
      case 1:
          //������ת��Ӻ���
          ek[0]  = (-set_speed_heng + set_speed_ratio*turn_speed) - encoder_data[0];//�������
          ek[2]  = ( set_speed_heng + set_speed_ratio*turn_speed) - encoder_data[2];//�������
          ek[1]  = ( set_speed_heng - set_speed_ratio*turn_speed) - encoder_data[1];//�ұ�����
          ek[3]  = (-set_speed_heng - set_speed_ratio*turn_speed) - encoder_data[3];//�ұ�����
          break;
      case 2:
          //������ת��Ӻ���
          ek[0]  = ( set_speed_heng + set_speed_ratio*turn_speed) - encoder_data[0];//�������
          ek[2]  = (-set_speed_heng + set_speed_ratio*turn_speed) - encoder_data[2];//�������
          ek[1]  = (-set_speed_heng - set_speed_ratio*turn_speed) - encoder_data[1];//�ұ�����
          ek[3]  = ( set_speed_heng - set_speed_ratio*turn_speed) - encoder_data[3];//�ұ�����
          break;
      default:
          break;
      }
  }
  //��תʱ
  else{
      switch(circle_pattern[p]){
      case 0:
          //������ת
          ek[0]  =  250 - encoder_data[0];
          ek[2]  =  250 - encoder_data[2];
          ek[1]  = -250 - encoder_data[1];
          ek[3]  = -250 - encoder_data[3];
          break;
      case 1:
          //������ת
          ek[0]  =  250 - encoder_data[0];
          ek[2]  =  250 - encoder_data[2];
          ek[1]  = -250 - encoder_data[1];
          ek[3]  = -250 - encoder_data[3];
          break;
      case 2:
          //������ת
          ek[0]  = -250 - encoder_data[0];
          ek[2]  = -250 - encoder_data[2];
          ek[1]  =  250 - encoder_data[1];
          ek[3]  =  250 - encoder_data[3];
          break;
      case 3:
          //������ת
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
      //ʹ�ú���ǰ������PID����
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
//  @brief          ת��������ƣ���speedx_power�źż�����ת���PWMռ�ձ�
//  @parameter      speedx_power��������ʽPID�ı���ֵ
//  @return         void
//  Sample usage:   Motor_Control(1000, 1000, 1000, 1000);4�������ת ����ռ�ձ�Ϊ �ٷ�֮ (speedx_power��1000/TIMER1_PWM_DUTY_MAX*100)
                                                         //�������ռ�ձ�ֵPWM_DUTY_MAX ������fsl_pwm.h�ļ����޸� Ĭ��Ϊ10000
//-------------------------------------------------------------------------------------------------------------------
void Motor_Control(int32 speed1_power, int32 speed2_power, int32 speed3_power, int32 speed4_power)
{
    if(0<=speed1_power) //���1   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/TIMER1_PWM_DUTY_MAX*100)
    {
        pwm_duty(MOTOR1_A, speed1_power);
        pwm_duty(MOTOR1_B, 0);
    }
    else                //���1   ��ת
    {
        pwm_duty(MOTOR1_A, 0);
        pwm_duty(MOTOR1_B, -speed1_power);
    }

    if(0<=speed2_power) //���2   ��ת
    {
        pwm_duty(MOTOR2_A, speed2_power);
        pwm_duty(MOTOR2_B, 0);
    }
    else                //���2   ��ת
    {
        pwm_duty(MOTOR2_A, 0);
        pwm_duty(MOTOR2_B, -speed2_power);
    }

    if(0<=speed3_power) //���3   ��ת
    {
        pwm_duty(MOTOR3_A, speed3_power);
        pwm_duty(MOTOR3_B, 0);
    }
    else                //���3   ��ת
    {
        pwm_duty(MOTOR3_A, 0);
        pwm_duty(MOTOR3_B, -speed3_power);
    }

    if(0<=speed4_power) //���3   ��ת
    {
        pwm_duty(MOTOR4_A, speed4_power);
        pwm_duty(MOTOR4_B, 0);
    }
    else                //���3   ��ת
    {
        pwm_duty(MOTOR4_A, 0);
        pwm_duty(MOTOR4_B, -speed4_power);
    }
}
