#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#include "oled_font.h"
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <string.h>


void OLED_WR_CMD(uint8_t cmd);
void OLED_WR_DATA(uint8_t data);
void OLED_Init(void);
void OLED_SetPos(uint8_t x,uint8_t y);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t ch,uint8_t size,uint8_t reverse);
void OLED_ShowString(uint8_t x,uint8_t y,char *str,uint8_t size,uint8_t reverse);
void OLED_Printf(int16_t X, int16_t Y, uint8_t reverse,uint8_t FontSize, char *format, ...);

// DMA相关函数
uint8_t OLED_Update_Async(void);
uint8_t OLED_DMA_IsReady(void);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);

#endif

