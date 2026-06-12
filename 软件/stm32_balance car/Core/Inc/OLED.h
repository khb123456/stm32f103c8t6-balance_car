#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// 字体大小选项
#define OLED_6X8    12
#define OLED_8X16   16

// 字模数组声明（在 OLED_Font.h 中定义）
extern const unsigned char F6x8[][6];
extern const unsigned char F8X16[];

void OLED_Init(void);
void OLED_Update(void);                     // 全屏刷新
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height); // 局部刷新
void OLED_Clear(void);
void OLED_DrawPoint(int16_t X, int16_t Y);
uint8_t OLED_GetPoint(int16_t X, int16_t Y);
void OLED_ShowChar(int16_t X, int16_t Y, char ch, uint8_t FontSize);
void OLED_ShowString(int16_t X, int16_t Y, char *str, uint8_t FontSize);
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...);

#endif
