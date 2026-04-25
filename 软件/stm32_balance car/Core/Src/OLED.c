#include "oled.h"
#include "i2c.h"
#include "string.h"

// DMA传输状态标志
volatile uint8_t OLED_DMA_TransferComplete = 1;

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

// ========== 显存缓冲区 ==========
uint8_t OLED_GRAM[8][128] = {0};  // 8页 × 128列
uint8_t OLED_Update_Flag = 0;     // 更新标志位

// ========== DMA传输缓冲区 ==========
#define OLED_BUF_SIZE 128  // 使用更大的缓冲区，一次传输整页数据
uint8_t OLED_DMA_Buf[OLED_BUF_SIZE + 1];  // 缓冲区+1字节的控制字节

// DMA传输控制结构
typedef struct {
    uint8_t page;
    uint8_t transfer_in_progress;
    uint8_t current_buffer;
} OLED_DMA_Control_t;

OLED_DMA_Control_t OLED_DMA_Ctrl = {0};
 
// OLED 初始化命令序列（来自控制器数据手册）
// 包含显示开关、时钟、复用率、对比度、预充电等设置
uint8_t CMD_Data[]={
0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,0xA1, 0xC8, 0xDA,

0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,0x8D, 0x14,

0xAF};

// DMA传输完成回调函数
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C1)
    {
        OLED_DMA_TransferComplete = 1;
        OLED_DMA_Ctrl.transfer_in_progress = 0;
    }
}

// 启动DMA传输
static uint8_t OLED_Start_DMA_Transfer(uint8_t control, uint8_t *data, uint16_t len)
{
    if(!OLED_DMA_TransferComplete || OLED_DMA_Ctrl.transfer_in_progress)
        return 0; // 上一次传输还未完成
    
    OLED_DMA_Buf[0] = control;
    if(len > OLED_BUF_SIZE) len = OLED_BUF_SIZE;
    memcpy(&OLED_DMA_Buf[1], data, len);
    
    OLED_DMA_TransferComplete = 0;
    OLED_DMA_Ctrl.transfer_in_progress = 1;
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(&hi2c1, OLED_ADDR, OLED_DMA_Buf, len + 1);
    
    if(status != HAL_OK)
    {
        OLED_DMA_TransferComplete = 1;
        OLED_DMA_Ctrl.transfer_in_progress = 0;
        return 0;
    }
    
    return 1;
}

// 检查DMA传输是否完成
uint8_t OLED_DMA_IsReady(void)
{
    return OLED_DMA_TransferComplete && !OLED_DMA_Ctrl.transfer_in_progress;
}

// 向 OLED 发送一个命令字节
void OLED_WR_CMD(uint8_t cmd)
{
    uint8_t buf[2] = {OLED_CMD, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 10);
}

// 向 OLED 发送一个数据字节（用于显示 RAM）
void OLED_WR_DATA(uint8_t data)
{
    uint8_t buf[2] = {OLED_DATA, data};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 10);
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

// 清屏：清空显存缓冲区
void OLED_Clear(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    OLED_Update_Flag = 1;
}

// 更新整个屏幕（DMA优化版本）
void OLED_Update(void)
{
    // 使用传统方式更新，避免DMA传输冲突
    uint8_t page, x;
    
    for(page = 0; page < 8; page++)
    {
        // 设置页面地址
        OLED_WR_CMD(0xB0 | page);  // 设置页地址
        OLED_WR_CMD(0x00);         // 设置列地址低4位
        OLED_WR_CMD(0x10);         // 设置列地址高4位
        
        // 批量传输该页的所有数据
        for(x = 0; x < 128; x += 32)  // 使用较小的块大小
        {
            uint16_t remain = 128 - x;
            uint16_t send_len = (remain > 32) ? 32 : remain;
            
            uint8_t buf[33];
            buf[0] = OLED_DATA;
            memcpy(&buf[1], &OLED_GRAM[page][x], send_len);
            HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, send_len + 1, 5);
        }
    }
    OLED_Update_Flag = 0;
}

// 异步更新屏幕（使用DMA）
uint8_t OLED_Update_Async(void)
{
    static uint8_t current_page = 0;
    static uint8_t update_started = 0;
    
    if(!update_started)
    {
        // 开始新的更新序列
        current_page = 0;
        update_started = 1;
    }
    
    if(OLED_DMA_IsReady())
    {
        if(current_page < 8)
        {
            // 设置页面地址
            OLED_WR_CMD(0xB0 | current_page);
            OLED_WR_CMD(0x00);
            OLED_WR_CMD(0x10);
            
            // 启动DMA传输整页数据
            if(OLED_Start_DMA_Transfer(OLED_DATA, OLED_GRAM[current_page], 128))
            {
                current_page++;
                return 1; // 传输进行中
            }
        }
        else
        {
            // 所有页面传输完成
            update_started = 0;
            OLED_Update_Flag = 0;
            return 2; // 传输完成
        }
    }
    
    return 0; // 等待传输完成
}

// 显示单个字符（写入显存缓冲区）
// x,y: 坐标（列, 页）；ch: ASCII 字符；size: 字体尺寸（12 或 16）；reverse: 反色
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t ch,uint8_t size,uint8_t reverse)
{
    uint8_t i;
    uint8_t page = y;
    ch -= ' ';
    
    if(x > 127) {x = 0; y += 2;}
    
    if(size == 12)
    {
        for(i = 0; i < 6; i++)
        {
            uint8_t data = F6x8[ch][i];
            if(reverse) data = ~data;
            if(x < 128 && page < 8)
                OLED_GRAM[page][x++] = data;
        }
    }
    else if(size == 16)
    {
        // 上半部分
        for(i = 0; i < 8; i++)
        {
            uint8_t data = F8X16[ch * 16 + i];
            if(reverse) data = ~data;
            if(x < 128 && page < 8)
                OLED_GRAM[page][x] = data;
        }
        
        // 下半部分
        x -= 8; // 回到起始位置
        for(i = 0; i < 8; i++)
        {
            uint8_t data = F8X16[ch * 16 + i + 8];
            if(reverse) data = ~data;
            if(x < 128 && (page + 1) < 8)
                OLED_GRAM[page + 1][x] = data;
            x++;
        }
    }
    OLED_Update_Flag = 1;
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

