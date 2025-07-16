// ���ļ��������Դ�����HMC5883L   SCL��B4 SDA��B6   ����    SCL B6   SCL B8

#include "zf_common_headfile.h"
#include "hmc5883.h"
#include <math.h>

#define PIT_CH                   (TIM2_PIT )                                      // ʹ�õ������жϱ�� ����޸� ��Ҫͬ����Ӧ�޸������жϱ���� isr.c �еĵ���
#define PIT_PRIORITY             (TIM2_IRQn)                                      // ��Ӧ�����жϵ��жϱ��
uint8 pit_state = 0;

extern int X,Y,Z;

void PIT2_Run(void)
{
    Multiple_Read_HMC5883();
}

int main(void) {
    clock_init(SYSTEM_CLOCK_144M);                                              // ��ʼ��оƬʱ�ӹ���Ƶ��Ϊ 144MHz
    debug_init();
    pit_ms_init(PIT_CH, 100);                                                   // ��ʼ�� PIT_CH0 Ϊ�����ж� 100ms ����
    interrupt_set_priority(PIT_PRIORITY, 0);                                    // ���� PIT2 �������жϵ��ж����ȼ�Ϊ 0
    HMC5883_Init();

    double Angle_XY=0,Angle_XZ=0,Angle_YZ=0;
    while(1)     //ѭ��
    {
        if(pit_state)
        {
           PIT2_Run();
           pit_state = 0;
        }
        //---------��ʾX��
        Angle_XY= atan2((double)Y,(double)X) * (180 / 3.14159265) + 180; //����XYƽ��Ƕ�
        Angle_XY*=10;

        Angle_XZ= atan2((double)Z,(double)X) * (180 / 3.14159265) + 180; //����XZƽ��Ƕ�
        Angle_XZ*=10;

        Angle_YZ= atan2((double)Z,(double)Y) * (180 / 3.14159265) + 180; //����YZƽ��Ƕ�
        Angle_YZ*=10;
        system_delay_ms(100);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     UART_INDEX �Ľ����жϴ����� ����������� UART_INDEX ��Ӧ���жϵ��� ��� isr.c
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (void)
{
     debug_interrupr_handler();
     gnss_uart_callback();
}

//-------------------------------------------------------------------------------------------------------------------
// �������     PIT �������жϴ����� ����������� PIT ��Ӧ�Ķ�ʱ���жϵ��� ��� isr.c
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
     pit_state = 1;                                                              // �����жϴ��� ��־λ��λ
}
