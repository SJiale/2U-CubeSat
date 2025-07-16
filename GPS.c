// 此文件用来调试GPS,ATGM336H芯片，采样RMC语句读取

#include "zf_common_headfile.h"
#include <math.h>
#include <Vofa.h>

#define UART_INDEX              (UART_1)                           // 默认 UART_1
#define UART_BAUDRATE           (9600)                           // 默认 9600
#define UART_TX_PIN             (UART1_MAP0_TX_A9)                           // 默认 UART_1_MAP0_TX_A9
#define UART_RX_PIN             (UART1_MAP0_RX_A10)                           // 默认 UART1_MAP0_RX_A10

#define UART_PRIORITY           (USART1_IRQn)                                   // 对应串口中断的中断编号 在 ch32v30x.h 头文件中查看 IRQn_Type 枚举体

uint8       pit_state = 0;
uint8       uart_get_data[64];                                                  // 串口接收数据缓冲区
uint8       fifo_get_data[64];                                                  // fifo 输出读出缓冲区
uint8       get_data = 0;                                                       // 接收数据变量
uint32      fifo_data_count = 0;                                                // fifo 数据个数

fifo_struct uart_data_fifo;

unsigned char byte[12];
unsigned char xbyte[4];
unsigned char ybyte[4];
unsigned char zbyte[4];
unsigned char sent_byte[4];

// **************************** 代码区域 ****************************
// Lora + GPS
//-------------------------------------------------------------------------------------------------------------------
void lora_Init(){
     uart_init(UART_3, 9600, UART3_MAP0_TX_B10, UART3_MAP0_RX_B11);
}

void Float_to_Byte (float f, unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata = f;
    byte[0] = (unsigned char) fl.ldata;
    byte[1] = (unsigned char) (fl.ldata >> 8);
    byte[2] = (unsigned char) (fl.ldata >> 16);
    byte[3] = (unsigned char) (fl.ldata >> 24);
}

void Vofa_JustFloat ()
{
//    Float_to_Byte(() * 1.0, xbyte);
//    Float_to_Byte(() * 1.0, ybyte);
//    Float_to_Byte(() * 1.0, zbyte);

    int i = 0;
    int k = 4;
    int m = 8;

    for (i = 0; i < 4; ++i) {
        byte[i] =  xbyte[i];
    }

    for (k = 4; k < 8; ++k) {
        byte[k] =  ybyte[k-4];
    }

    for (m = 8; m < 12; ++m) {
        byte[m] = zbyte[m-8];
    }

    for (int t_test = 0; t_test < 12; t_test++)
    {
        uart_write_byte(UART_3, byte[t_test]);         //向串口3发送数据
    }

    sent_byte[0] = 0X00;
    sent_byte[1] = 0X00;
    sent_byte[2] = 0X80;
    sent_byte[3] = 0X7f;

    for (int t_test = 0; t_test < 4; t_test++)
    {
        uart_write_byte(UART_3, sent_byte[t_test]);         //向串口3发送帧尾
    }
}

int main (void){
    // 此处编写用户代码 例如外设初始化代码等
    clock_init(SYSTEM_CLOCK_144M);                                              // 初始化芯片时钟工作频率为 144MHz
    debug_init();

    interrupt_set_priority(UART_PRIORITY, (0<<5) || 1);                         // 设置对应 UART_INDEX 的中断抢占优先级0，子优先级1// 初始化默认 Debug UART
    lora_Init();
    gnss_init(TAU1201);

    while(1)
    {
       // 此处编写需要循环执行的代码
       gnss_data_parse();
       Vofa_JustFloat ();
       system_delay_ms(10);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (void)
{
    debug_interrupr_handler();
    gnss_uart_callback();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    pit_state = 1;                                                              // 周期中断触发 标志位置位
}
