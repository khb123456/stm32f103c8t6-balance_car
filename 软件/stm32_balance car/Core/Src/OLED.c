#include "oled.h"
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
// I2C 地址与控制字节
#define OLED_ADDR 0x78
#define OLED_CMD  0x00
#define OLED_DATA 0x40
 
// OLED 初始化命令序列（来自控制器数据手册）
// 包含显示开关、时钟、复用率、对比度、预充电等设置
uint8_t CMD_Data[]={
0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,0xA1, 0xC8, 0xDA,

0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,0x8D, 0x14,

0xAF};

// 向 OLED 发送一个命令字节
void OLED_WR_CMD(uint8_t cmd)
{
    i2c_write_byte(OLED_ADDR,OLED_CMD,cmd);
}

// 向 OLED 发送一个数据字节（用于显示 RAM）
void OLED_WR_DATA(uint8_t data)
{
    i2c_write_byte(OLED_ADDR,OLED_DATA,data);
}

// 初始化 OLED：发送初始化命令并清屏
void OLED_Init(void)
{
    for(int i=0;i<sizeof(CMD_Data);i++)
    {
        OLED_WR_CMD(CMD_Data[i]);
    }
    OLED_Clear();
}

// 设置光标位置：x 列（0-127），y 页（0-7）
void OLED_SetPos(uint8_t x,uint8_t y)
{
    OLED_WR_CMD(0xB0|y);
    OLED_WR_CMD(x&0x0F);
    OLED_WR_CMD(0x10|(x>>4));
}

// 清屏：将显示 RAM 全部写为 0
void OLED_Clear(void)
{
    uint8_t x,y;
    for(y=0;y<8;y++)
    {
        OLED_WR_CMD(0xB0+y);
        OLED_WR_CMD(0x00);
        OLED_WR_CMD(0x10);
        for(x=0;x<128;x++)
        {
            OLED_WR_DATA(0x00);
        }
    }
}

// 显示单个字符
// x,y: 坐标（列, 页）；ch: ASCII 字符；size: 字体尺寸（12 或 16）；reverse: 反色
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t ch,uint8_t size,uint8_t reverse)
{
    uint8_t i;
    ch-=' ';
    if(x>127) {x=0;y+=2;}
    if(size==12)
    {
        OLED_SetPos(x,y);
        for(i=0;i<6;i++)
        {
            if(reverse)
                OLED_WR_DATA(F6x8[ch][i]);
            else
                OLED_WR_DATA(F6x8[ch][i]);
        }
    }
    if(size==16)
    {
        OLED_SetPos(x,y);
        for(i=0;i<8;i++)
        {
            if(reverse)
                OLED_WR_DATA(~F8X16[ch*16+i]);
            else
                OLED_WR_DATA(F8X16[ch*16+i]);
        }
        OLED_SetPos(x,y+1);
        for(i=0;i<8;i++)
        {
            if(reverse)
                OLED_WR_DATA(~F8X16[ch*16+i+8]);
            else
                OLED_WR_DATA(F8X16[ch*16+i+8]);
        }
    }

}

// 显示字符串，从 (x,y) 开始，遇到到达行尾会换页
void OLED_ShowString(uint8_t x,uint8_t y,char *str,uint8_t size,uint8_t reverse)
{
    uint8_t j=0;
    while(str[j]!='\0')
    {
        OLED_ShowChar(x,y,str[j],size,reverse);
        if(size==12)
            x+=6;
        else
            x+=8;
        if(x>122&&size==12)
        {
            x=0;
            y++;
        }
        if(x>120&&size==16)
        {
            x=0;
            y++;
        }
        j++;
    }
}
void OLED_Printf(int16_t X, int16_t Y, uint8_t reverse,uint8_t FontSize, char *format, ...)
{
	char String[256];						//定义字符数组
	va_list arg;							//定义可变参数列表数据类型的变量arg
	va_start(arg, format);					//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);			//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);							//结束变量arg
	OLED_ShowString(X, Y, String, FontSize,reverse);//OLED显示字符数组（字符串）
}

