#include "encoder.h"
#include "headfile.h"

uint16 encoder_data[4];//��Ϊ�ײ����ʵ�֣�ֱ������ٶ�

void Encoder_Init(void)
{
    encoder_init_spi(ABS_ENCODER_SPI_PC1_PIN);      //������1��ʼ����
    encoder_init_spi(ABS_ENCODER_SPI_PC2_PIN);      //������2��ʼ����
    encoder_init_spi(ABS_ENCODER_SPI_PC3_PIN);      //������3��ʼ����
    encoder_init_spi(ABS_ENCODER_SPI_PC4_PIN);      //������4��ʼ����
}


void encoder_get(void)
{
    encoder_data[0] = -encoder1_speed_spi(ABS_ENCODER_SPI_PC1_PIN);          //��ȡ������1������
    encoder_data[1] =  encoder2_speed_spi(ABS_ENCODER_SPI_PC2_PIN);          //��ȡ������2������
    encoder_data[2] = -encoder3_speed_spi(ABS_ENCODER_SPI_PC3_PIN);          //��ȡ������3������
    encoder_data[3] =  encoder4_speed_spi(ABS_ENCODER_SPI_PC4_PIN);          //��ȡ������4������
}

