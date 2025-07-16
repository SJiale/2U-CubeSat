// ���ļ���������GPS,ATGM336HоƬ������RMC����ȡ

#include "zf_common_headfile.h"
#include <math.h>
#include <Vofa.h>

#define UART_INDEX              (UART_1)                           // Ĭ�� UART_1
#define UART_BAUDRATE           (9600)                           // Ĭ�� 9600
#define UART_TX_PIN             (UART1_MAP0_TX_A9)                           // Ĭ�� UART_1_MAP0_TX_A9
#define UART_RX_PIN             (UART1_MAP0_RX_A10)                           // Ĭ�� UART1_MAP0_RX_A10

#define UART_PRIORITY           (USART1_IRQn)                                   // ��Ӧ�����жϵ��жϱ�� �� ch32v30x.h ͷ�ļ��в鿴 IRQn_Type ö����

uint8       pit_state = 0;
uint8       uart_get_data[64];                                                  // ���ڽ������ݻ�����
uint8       fifo_get_data[64];                                                  // fifo �������������
uint8       get_data = 0;                                                       // �������ݱ���
uint32      fifo_data_count = 0;                                                // fifo ���ݸ���

fifo_struct uart_data_fifo;

unsigned char byte[12];
unsigned char xbyte[4];
unsigned char ybyte[4];
unsigned char zbyte[4];
unsigned char sent_byte[4];

// **************************** �������� ****************************
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
        uart_write_byte(UART_3, byte[t_test]);         //�򴮿�3��������
    }

    sent_byte[0] = 0X00;
    sent_byte[1] = 0X00;
    sent_byte[2] = 0X80;
    sent_byte[3] = 0X7f;

    for (int t_test = 0; t_test < 4; t_test++)
    {
        uart_write_byte(UART_3, sent_byte[t_test]);         //�򴮿�3����֡β
    }
}

int main (void){
    // �˴���д�û����� ���������ʼ�������
    clock_init(SYSTEM_CLOCK_144M);                                              // ��ʼ��оƬʱ�ӹ���Ƶ��Ϊ 144MHz
    debug_init();

    interrupt_set_priority(UART_PRIORITY, (0<<5) || 1);                         // ���ö�Ӧ UART_INDEX ���ж���ռ���ȼ�0�������ȼ�1// ��ʼ��Ĭ�� Debug UART
    lora_Init();
    gnss_init(TAU1201);

    while(1)
    {
       // �˴���д��Ҫѭ��ִ�еĴ���
       gnss_data_parse();
       Vofa_JustFloat ();
       system_delay_ms(10);
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
