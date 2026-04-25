#include "MyI2C.h"
//void HAL_Delay_us(uint32_t us)
//{
//	uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000) * us;
//    uint32_t start = SysTick->VAL;
//    while((start - SysTick->VAL) < ticks);
//}
void IIC_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, (GPIO_PinState)BitValue);     
	HAL_Delay_us(5);
}

void IIC_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, (GPIO_PinState)BitValue); 
	HAL_Delay_us(5);
}


uint8_t IIC_R_SDA(void)
{
    uint8_t BitValue;
    BitValue = (uint8_t)HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN); 
    HAL_Delay_us(5);                                               
    return BitValue;                                           
}
void IIC_Init(void)
{
	IIC_W_SDA(1);                            
    IIC_W_SCL(1);
	HAL_Delay(100);
}

void IIC_Start(void)
{
    IIC_W_SDA(1);                            
    IIC_W_SCL(1); 
	HAL_Delay_us(5);
    IIC_W_SDA(0);  
	HAL_Delay_us(5);
    IIC_W_SCL(0);                             
}

void IIC_Stop(void)
{
    IIC_W_SDA(0);                            
    IIC_W_SCL(1);   
	HAL_Delay_us(5);
    IIC_W_SDA(1);    
	HAL_Delay_us(5);
}

void IIC_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)                    
    {
        IIC_W_SCL(0); 
        IIC_W_SDA(!!(Byte & 0x80)); 
		Byte<<=1;
        IIC_W_SCL(1);                                    
    }
	IIC_W_SCL(0);
	IIC_W_SDA(1);
	IIC_ReceiveAck();
}

uint8_t IIC_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;                     
    IIC_W_SDA(1);                             
    for (i = 0; i < 8; i++)                     
    {
        IIC_W_SCL(1);                         
        if (IIC_R_SDA()) { Byte |= (0x80 >> i); }                                          
        IIC_W_SCL(0);                         
    }
    return Byte;                                
}

void IIC_SendAck(uint8_t AckBit)
{
	IIC_W_SCL(0);
    IIC_W_SDA(AckBit);                       
    IIC_W_SCL(1);     
	HAL_Delay_us(5);
    IIC_W_SCL(0);                             
}

uint8_t IIC_ReceiveAck(void)
{
    uint8_t AckBit;                             
    IIC_W_SDA(1);                            
    IIC_W_SCL(1);                            
    AckBit = IIC_R_SDA();                    
    IIC_W_SCL(0);                             
    return AckBit;                               
}

