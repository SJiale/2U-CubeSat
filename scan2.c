#include "zf_common_headfile.h"
#include "MahonyAHRS.h"
#include "hmc5883.h"
#include <math.h>
#include <Vofa.h>

// **************************** 代码区域 ****************************
#define UART_INDEX              (UART_3)                        // 默认 UART_3
#define UART_BAUDRATE           (115200)                           // 默认 115200
#define UART_TX_PIN             (UART3_MAP0_TX_B10)                           // 默认 UART3_MAP0_TX_B10
#define UART_RX_PIN             (UART3_MAP0_RX_B11)                           // 默认 UART3_MAP0_RX_B11

#define UART_PRIORITY           (USART3_IRQn)                                   // 对应串口中断的中断编号 在 ch32v30x.h 头文件中查看 IRQn_Type 枚举体
#define PWM_CH1                 (TIM4_PWM_MAP1_CH1_D12)
#define PWM_CH2                 (TIM4_PWM_MAP1_CH2_D13)
#define PWM_CH3                 (TIM4_PWM_MAP1_CH3_D14)
uint8       uart_get_data[64];                                                  // 串口接收数据缓冲区
uint8       fifo_get_data[64];                                                  // fifo 输出读出缓冲区

uint8       get_data = 0;                                                       // 接收数据变量
uint32      fifo_data_count = 0;                                                // fifo 数据个数

fifo_struct uart_data_fifo;

#define PIT_CH                   (TIM2_PIT )                                      // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY             (TIM2_IRQn)                                      // 对应周期中断的中断编号

#define LED2                    (B4)

float fgx, fgy, fgz, fax, fay, faz;
float hxj, fyj, hgj, fyj_a0, fyj_a1;  // 姿态角
unsigned char byte[12];
unsigned char xbyte[4];
unsigned char ybyte[4];
unsigned char zbyte[4];
unsigned char testbyte[4];
unsigned char test1byte[4];
uint16_t Gyro[3];
uint8 DAT;
unsigned char sent_byte[4];
unsigned char sent1_byte[4];
char Gyrox[25];
char Gyroy[25];
char Gyroz[25];

fifo_struct uart_data_fifo;

uint8 pit_state = 0;

// **************************** 代码区域 ****************************
// lora模块
// -------------------------------------------------------------------------------------------------------------------
void lora_Init(){
     uart_init(UART_3, 115200, UART3_MAP0_TX_B10, UART3_MAP0_RX_B11);
}

void Float_to_Byte (float f, unsigned char byte[])     // 浮点数转四字节
{
    FloatLongType fl;
    fl.fdata = f;
    byte[0] = (unsigned char) fl.ldata;
    byte[1] = (unsigned char) (fl.ldata >> 8);
    byte[2] = (unsigned char) (fl.ldata >> 16);
    byte[3] = (unsigned char) (fl.ldata >> 24);
}

//void Vofa_JustFloat ()
//{
//    Float_to_Byte((fyj/100 + 29 ) * 1.0, xbyte);
//    Float_to_Byte((hgj/100 - 15 ) * 1.0, ybyte);
//    Float_to_Byte((hxj/100 + 25 ) * 1.0, zbyte);
//
//    int i = 0;
//    int k = 4;
//    int m = 8;
//
//    for (i = 0; i < 4; ++i) {
//        byte[i] =  xbyte[i];
//    }
//
//    for (k = 4; k < 8; ++k) {
//        byte[k] =  ybyte[k-4];
//    }
//
//    for (m = 8; m < 12; ++m) {
//        byte[m] = zbyte[m-8];
//    }
//
//    for (int t_test = 0; t_test < 12; t_test++)
//    {
//        uart_write_byte(UART_3, byte[t_test]);         //向串口1发送数据
//    }
//
//    sent_byte[0] = 0X00;
//    sent_byte[1] = 0X00;
//    sent_byte[2] = 0X80;
//    sent_byte[3] = 0X7f;
//
//    for (int t_test = 0; t_test < 4; t_test++)
//    {
//        uart_write_byte(UART_3, sent_byte[t_test]);         //向串口1发送帧尾
//    }
//}

//-------------------------------------------------------------------------------------------------------------------
// 在 TIM2 中断获取mpu6050数据、编码器数值、编码器速度 并计算
//-------------------------------------------------------------------------------------------------------------------

void PIT2_Run(void)
{
    static int cnt=0;
    cnt++;
    if(cnt>=9)
    {
        cnt=0;
        gpio_toggle_level(LED2);
    }
        mpu6050_get_acc();
        mpu6050_get_gyro();

        fgx=-mpu6050_gyro_x*0.00106414676712212148299493466139;
        fgy=-mpu6050_gyro_y*0.00106414676712212148299493466139;
        fgz= mpu6050_gyro_z*0.00106414676712212148299493466139;
        fax=-mpu6050_acc_x*0.000244140625;
        fay=-mpu6050_acc_y*0.000244140625;
        faz= mpu6050_acc_z*0.000244140625;

        MahonyAHRSupdateIMU(fgx,fgy,fgz,fax,fay,faz);

        fyj=asinf(2*(q0*q2-q1*q3))*5730;//俯仰角
        hgj=atan2f((q0*q1+q2*q3),(1-2*(q1*q1+q2*q2)))*5730;
        hxj=atan2f((q0*q3+q1*q2),(1-2*(q2*q2+q3*q3)))*5730;
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_144M);                                         // 初始化芯片时钟 工作频率为 144MHz
    debug_init();                                                          // 初始化默认 Debug UART

    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);                         // 初始化 LED1 输出 默认低电平 推挽输出模式

    pit_ms_init(PIT_CH, 100);                                              // 初始化 PIT_CH0 为周期中断 100ms 周期
    interrupt_set_priority(PIT_PRIORITY, 0);
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
    lora_Init();
    pwm_init (PWM_CH1, 17000, 7200);     // 修改初始 PWM 占空比        在实验台上控制绕X轴的旋转
    pwm_init (PWM_CH2, 17000, 7200);     // 对应Z轴旋转
    pwm_init (PWM_CH3, 17000, 7200);      // 对应Y轴旋转
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_PRIORITY, (0<<5) || 1);                         // 设置对应 UART_INDEX 的中断抢占优先级0，子优先级1

    mpu6050_init();

    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        // 此处编写需要循环执行的代码
        fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
        if(fifo_data_count != 0)                                                // 读取到数据了
        {
           fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
           uart_write_string(UART_INDEX, "UART get data:");
           uart_write_buffer(UART_INDEX, fifo_get_data, fifo_data_count);
           if (fifo_data_count == 1) {
               pwm_set_duty (PWM_CH1, 6500);
               pwm_set_duty (PWM_CH2, 6500);
               pwm_set_duty (PWM_CH3, 6500);
    //         Vofa_JustFloat1();
               pit_disable(TIM2_PIT);
           }
           if (fifo_data_count == 2) {
               pit_enable(TIM2_PIT);
               pwm_set_duty (PWM_CH1, 6000);
               pwm_set_duty (PWM_CH2, 6000);
               pwm_set_duty (PWM_CH3, 6000);
           }
           // 此处编写立方星扫描代码
        }
        system_delay_ms(10);
        if(pit_state)
        {
           PIT2_Run();
           pit_state = 0;                                                      // 清空周期中断触发标志位
        }
        // 此处编写需要循环执行的代码
////    Vofa_JustFloat();
//    system_delay_ms(1);
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
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // 将数据写入 fifo 中
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT2 的周期中断处理函数 这个函数将在 PIT2 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    pit_state = 1;                                                          // 周期中断触发 标志位置位
}
