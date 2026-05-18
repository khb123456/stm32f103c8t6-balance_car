#ifndef __I2C_BUS_H
#define __I2C_BUS_H

#include "stm32f1xx_hal.h"

//I2C总线模式切换：0-软件模拟，1-硬件I2C1
#define USE_HW_I2C 1    

void i2c_init(void);
void i2c_write_byte(uint8_t addr,uint8_t cmd,uint8_t byte);


#endif
