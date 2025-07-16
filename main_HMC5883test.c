// 此文件用来调试磁力计HMC5883L   SCL接B4 SDA接B6   换成    SCL B6   SCL B8

#include "zf_common_headfile.h"
#include "hmc5883.h"
#include <math.h>

#define PIT_CH                   (TIM2_PIT )                                      // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY             (TIM2_IRQn)                                      // 对应周期中断的中断编号
uint8 pit_state = 0;

extern int X,Y,Z;

void PIT2_Run(void)
{
    Multiple_Read_HMC5883();
}

int main(void) {
    clock_init(SYSTEM_CLOCK_144M);                                              // 初始化芯片时钟工作频率为 144MHz
    debug_init();
    pit_ms_init(PIT_CH, 100);                                                   // 初始化 PIT_CH0 为周期中断 100ms 周期
    interrupt_set_priority(PIT_PRIORITY, 0);                                    // 设置 PIT2 对周期中断的中断优先级为 0
    HMC5883_Init();

    double Angle_XY=0,Angle_XZ=0,Angle_YZ=0;
    while(1)     //循环
    {
        if(pit_state)
        {
           PIT2_Run();
           pit_state = 0;
        }
        //---------显示X轴
        Angle_XY= atan2((double)Y,(double)X) * (180 / 3.14159265) + 180; //计算XY平面角度
        Angle_XY*=10;

        Angle_XZ= atan2((double)Z,(double)X) * (180 / 3.14159265) + 180; //计算XZ平面角度
        Angle_XZ*=10;

        Angle_YZ= atan2((double)Z,(double)Y) * (180 / 3.14159265) + 180; //计算YZ平面角度
        Angle_YZ*=10;
        system_delay_ms(100);
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
