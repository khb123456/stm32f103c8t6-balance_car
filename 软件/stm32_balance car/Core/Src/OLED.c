/**********************************************************
 根据数据手册可以查到，此数组为命令数据：
0xAE：关闭显示器。
0xD5 0x80：设置显示时钟分频因子/振荡器频率。
0xA8 0x3F：设置多路复用率（1/64）。
0xD3 0x00：设置显示偏移（无偏移）。
0x40：设置起始行为0。
0xA1：设置段重定向（列地址0映射到SEG0）。
0xC8：设置COM扫描方向（上到下）。
0xDA 0x12：设置COM硬件引脚配置。
0x81 0xCF：设置对比度控制。
0xD9 0xF1：设置预充电周期。
0xDB 0x40：设置VCOMH电压倍率。
0xA4：设置显示全部点亮。
0xA6：设置显示正常（非反相）模式。
0x8D 0x14：设置DC-DC电荷泵使能和电荷泵倍率。
0xAF：打开显示器，开始显示。
 ***********************************************************/
#include "oled.h"
#include "i2c.h"              // 包含 I2C 句柄（如 extern I2C_HandleTypeDef hi2c1;）
#include "OLED_Font.h"        // 你的字模数组定义
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define OLED_ADDR   0x78
#define OLED_CMD    0x00
#define OLED_DATA   0x40

// 显存缓冲区：8页 × 128列
uint8_t OLED_DisplayBuf[8][128];

// 初始化命令序列
static const uint8_t InitCmd[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
    0xDB, 0x40, 0xA4, 0xA6, 0x8D, 0x14, 0xAF
};

// 批量 I2C 发送（控制字节 + 数据）
static void I2C_WriteMulti(uint8_t addr, uint8_t ctrl, uint8_t *data, uint16_t len)
{
    uint8_t buf[129];
    buf[0] = ctrl;
    memcpy(buf + 1, data, len);
    HAL_I2C_Master_Transmit(&hi2c1, addr, buf, len + 1, 100);
}

// 写命令
static void OLED_WriteCommand(uint8_t cmd)
{
    I2C_WriteMulti(OLED_ADDR, OLED_CMD, &cmd, 1);
}

// 写多个数据字节
static void OLED_WriteDataBlock(uint8_t *data, uint16_t len)
{
    I2C_WriteMulti(OLED_ADDR, OLED_DATA, data, len);
}

// 设置光标位置（x:列 0~127, y:页 0~7）
static void OLED_SetCursor(uint8_t x, uint8_t y)
{
    OLED_WriteCommand(0xB0 | y);
    OLED_WriteCommand(0x10 | ((x >> 4) & 0x0F));
    OLED_WriteCommand(0x00 | (x & 0x0F));
}

// 初始化 OLED
void OLED_Init(void)
{
    for (uint8_t i = 0; i < sizeof(InitCmd); i++) {
        OLED_WriteCommand(InitCmd[i]);
    }
    OLED_Clear();
    OLED_Update();
}

// 全屏刷新（发送全部8页）
void OLED_Update(void)
{
    for (uint8_t page = 0; page < 8; page++) {
        OLED_SetCursor(0, page);
        OLED_WriteDataBlock(OLED_DisplayBuf[page], 128);
    }
}

// 局部刷新：只更新指定矩形区域（X, Y, Width, Height 单位：像素）
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    if (X < 0) { Width += X; X = 0; }
    if (Y < 0) { Height += Y; Y = 0; }
    if (X >= OLED_WIDTH || Y >= OLED_HEIGHT) return;
    if (X + Width > OLED_WIDTH) Width = OLED_WIDTH - X;
    if (Y + Height > OLED_HEIGHT) Height = OLED_HEIGHT - Y;
    if (Width == 0 || Height == 0) return;

    int16_t start_page = Y / 8;
    int16_t end_page = (Y + Height - 1) / 8;
    if (start_page < 0) start_page = 0;
    if (end_page >= 8) end_page = 7;

    for (int16_t page = start_page; page <= end_page; page++) {
        // 计算本页内需要更新的列范围（因为不同页的X范围相同）
        uint8_t start_col = X;
        uint8_t end_col = X + Width - 1;
        if (start_col >= OLED_WIDTH) continue;
        if (end_col >= OLED_WIDTH) end_col = OLED_WIDTH - 1;
        uint8_t col_len = end_col - start_col + 1;

        // 设置光标到本页的起始列
        OLED_SetCursor(start_col, page);
        // 发送本页中从 start_col 开始的 col_len 个字节
        OLED_WriteDataBlock(&OLED_DisplayBuf[page][start_col], col_len);
    }
}

// 清空显存
void OLED_Clear(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        memset(OLED_DisplayBuf[i], 0x00, 128);
    }
}

// 画点（支持任意坐标，自动定位到页和位）
void OLED_DrawPoint(int16_t X, int16_t Y)
{
    if (X < 0 || X >= OLED_WIDTH || Y < 0 || Y >= OLED_HEIGHT) return;
    uint8_t page = Y / 8;
    uint8_t bit = Y % 8;
    OLED_DisplayBuf[page][X] |= (1 << bit);
}

// 读点
uint8_t OLED_GetPoint(int16_t X, int16_t Y)
{
    if (X < 0 || X >= OLED_WIDTH || Y < 0 || Y >= OLED_HEIGHT) return 0;
    uint8_t page = Y / 8;
    uint8_t bit = Y % 8;
    return (OLED_DisplayBuf[page][X] >> bit) & 0x01;
}

// 显示单个字符（支持任意Y坐标，自动跨页绘制）
void OLED_ShowChar(int16_t X, int16_t Y, char ch, uint8_t FontSize)
{
    if (X < 0 || X >= OLED_WIDTH) return;
    if (Y < 0 || Y >= OLED_HEIGHT) return;

    uint8_t idx = (uint8_t)(ch - ' ');
    if (idx >= 95) idx = 0;

    uint8_t i, j;
    if (FontSize == OLED_6X8) {
        const unsigned char *pFont = F6x8[idx];
        for (i = 0; i < 6; i++) {
            if (X + i >= OLED_WIDTH) break;
            uint8_t data = pFont[i];
            for (j = 0; j < 8; j++) {
                if (data & (1 << j)) {
                    OLED_DrawPoint(X + i, Y + j);
                }
            }
        }
    }
    else if (FontSize == OLED_8X16) {
        const unsigned char *pFont = &F8X16[idx * 16];
        for (i = 0; i < 8; i++) {
            if (X + i >= OLED_WIDTH) break;
            uint8_t data_upper = pFont[i];
            uint8_t data_lower = pFont[i + 8];
            // 上半部分
            for (j = 0; j < 8; j++) {
                if (data_upper & (1 << j)) {
                    OLED_DrawPoint(X + i, Y + j);
                }
            }
            // 下半部分
            for (j = 0; j < 8; j++) {
                if (data_lower & (1 << j)) {
                    OLED_DrawPoint(X + i, Y + 8 + j);
                }
            }
        }
    }
}

// 显示字符串（Y 为像素坐标，自动换行）
void OLED_ShowString(int16_t X, int16_t Y, char *str, uint8_t FontSize)
{
    uint8_t width = (FontSize == OLED_6X8) ? 6 : 8;
    uint8_t height = (FontSize == OLED_6X8) ? 8 : 16;
    while (*str) {
        OLED_ShowChar(X, Y, *str++, FontSize);
        X += width;
        if (X + width > OLED_WIDTH) {
            X = 0;
            Y += height;
            if (Y >= OLED_HEIGHT) break;
        }
    }
}

// 格式化输出（Y 参数支持行号 0~7 自动转换为像素坐标）
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
    // 如果 Y 在 0~7 范围内且字体是 6x8（高度8），则视为行号转换为像素坐标
    if (FontSize == OLED_6X8 && Y >= 0 && Y <= 7) {
        Y = Y * 8;
    }
    // 如果字体是 8x16，Y 在 0~3 范围内视为行号（因为高度16，屏幕最多4行）
    else if (FontSize == OLED_8X16 && Y >= 0 && Y <= 3) {
        Y = Y * 16;
    }

    char buf[128];
    va_list args;
    va_start(args, format);
    vsprintf(buf, format, args);
    va_end(args);
    OLED_ShowString(X, Y, buf, FontSize);
}
