#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f1xx_hal.h"
#include "sr04.h"
#define SCL_PIN  GPIO_PIN_4
#define SCL_PORT GPIOB
#define SDA_PIN  GPIO_PIN_3
#define SDA_PORT GPIOB

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(uint8_t Byte);
uint8_t IIC_ReceiveByte(void);
void IIC_SendAck(uint8_t AckBit);
uint8_t IIC_ReceiveAck(void);

#endif

