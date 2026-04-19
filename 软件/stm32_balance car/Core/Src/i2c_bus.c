#include "i2c_bus.h"

#if USE_HW_I2C
#include "i2c.h"
void i2c_init(void)
{
    HAL_Delay(100);
}

void i2c_write_byte(uint8_t addr,uint8_t cmd,uint8_t byte)
{
    HAL_I2C_Mem_Write(&hi2c1,addr,cmd,I2C_MEMADD_SIZE_8BIT,&byte,1,100);
}

#else
#include "MyI2C.h"
void i2c_init(void)
{
    IIC_Init();
}

void i2c_write_byte(uint8_t addr,uint8_t cmd,uint8_t byte)
{
    IIC_Start();
	IIC_SendByte(addr);
	IIC_SendByte(cmd);
	IIC_SendByte(byte);
	IIC_Stop();
}

#endif
