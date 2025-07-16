// 此文件用来测试lora模块是否回传数据

#include "zf_common_headfile.h"
#include "MahonyAHRS.h"
#include <math.h>
#include <Vofa.h>
#define PIT_CH                   (TIM2_PIT )                                      // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY             (TIM2_IRQn)                                      // 对应周期中断的中断编号
float fgx, fgy, fgz, fax, fay, faz;
float hxj, fyj, hgj, fyj_a0, fyj_a1;  // 姿态角
uint8 pit_state = 0;
unsigned char byte[12];
unsigned char xbyte[4];
unsigned char ybyte[4];
unsigned char zbyte[4];
uint16_t Gyro[3];
unsigned char sent_byte[4];
char Gyrox[25];
char Gyroy[25];
char Gyroz[25];

// **************************** 代码区域 ****************************
// lora模块
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
//    byte[4] = (unsigned char) (fl.ldata >> 32);
//    byte[5] = (unsigned char) (fl.ldata >> 40);
//    byte[6] = (unsigned char) (fl.ldata >> 48);
//    byte[7] = (unsigned char) (fl.ldata >> 56);
//    byte[8] = (unsigned char) (fl.ldata >> 64);
//    byte[9] = (unsigned char) (fl.ldata >> 72);
//    byte[10] = (unsigned char) (fl.ldata >> 80);
//    byte[11] = (unsigned char) (fl.ldata >> 88);
}

void Vofa_JustFloat ()
{
    Float_to_Byte((mpu6050_gyro_x + 29 ) * 1.0, xbyte);
    Float_to_Byte((mpu6050_gyro_y - 15 ) * 1.0, ybyte);
    Float_to_Byte((mpu6050_gyro_z + 25 ) * 1.0, zbyte);

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
        uart_write_byte(UART_3, byte[t_test]);         //向串口1发送数据
    }

//    Float_to_Byte(mpu6050_gyro_y * 1.0, byte);
//    for (int h_test = 5; h_test < 9; h_test++)
//    {
//        uart_write_byte(UART_3, byte[h_test]);         //向串口1发送数据
//    }
//
//    Float_to_Byte(mpu6050_gyro_z * 1.0, byte);
//    for (int k_test = 9; k_test < 12; k_test++)
//    {
//        uart_write_byte(UART_3, byte[k_test]);         //向串口1发送数据
//    }

    sent_byte[0] = 0X00;
    sent_byte[1] = 0X00;
    sent_byte[2] = 0X80;
    sent_byte[3] = 0X7f;

    for (int t_test = 0; t_test < 4; t_test++)
    {
        uart_write_byte(UART_3, sent_byte[t_test]);         //向串口1发送帧尾
    }
}

//void lorasendbackdata(){
//////      uart_write_string(UART_3, 'X轴陀螺仪数据:');
////      uart_write_string(UART_3, Gyrox);
//////      uart_write_string(UART_3, 'Y轴陀螺仪数据:');
////      uart_write_string(UART_3, Gyroy);
//////      uart_write_string(UART_3, 'Z轴陀螺仪数据:');
////      uart_write_string(UART_3, Gyroz);
//    uart_write_byte(UART_3, 0x41);
//}

void PIT2_Run(void)
{
        mpu6050_get_acc();
        mpu6050_get_gyro();
        fgx=-mpu6050_gyro_x*0.00106414676712212148299493466139;
        fgy=-mpu6050_gyro_y*0.00106414676712212148299493466139;
        fgz= mpu6050_gyro_z*0.00106414676712212148299493466139;
        fax=-mpu6050_acc_x*0.000244140625;
        fay=-mpu6050_acc_y*0.000244140625;
        faz= mpu6050_acc_z*0.000244140625;
        MahonyAHRSupdateIMU(fgx,fgy,fgz,fax,fay,faz);
        fyj=asinf(2*(q0*q2-q1*q3))*5730;  //俯仰角
        hgj=atan2f((q0*q1+q2*q3),(1-2*(q1*q1+q2*q2)))*5730;
        hxj=atan2f((q0*q3+q1*q2),(1-2*(q2*q2+q3*q3)))*5730;

        sprintf(Gyrox, " %d" , mpu6050_gyro_x + 30);
        sprintf(Gyroy, " %d" , mpu6050_gyro_y - 18);
        sprintf(Gyroz, " %d" , mpu6050_gyro_z + 25);
}

int main (void){
    clock_init(SYSTEM_CLOCK_144M);                                              // 初始化芯片时钟工作频率为 144MHz
    debug_init();                                                               // 初始化默认 Debug UART
    pit_ms_init(PIT_CH, 10);                                                   // 初始化 PIT_CH2 为周期中断 100ms 周期
    interrupt_set_priority(PIT_PRIORITY, 0);                                    // 设置 PIT2 对周期中断的中断优先级为 0
    lora_Init();
    mpu6050_init();

    while(1){
        if (pit_state) {
            PIT2_Run();
        }
    Vofa_JustFloat ();
    system_delay_ms(1);
}
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
