#include "zf_common_headfile.h"
#include "MahonyAHRS.h"
#include "hmc5883.h"
#include <math.h>
#include <Vofa.h>

// **************************** �������� ****************************
#define UART_INDEX              (UART_3)                        // Ĭ�� UART_3
#define UART_BAUDRATE           (115200)                           // Ĭ�� 115200
#define UART_TX_PIN             (UART3_MAP0_TX_B10)                           // Ĭ�� UART3_MAP0_TX_B10
#define UART_RX_PIN             (UART3_MAP0_RX_B11)                           // Ĭ�� UART3_MAP0_RX_B11

#define UART_PRIORITY           (USART3_IRQn)                                   // ��Ӧ�����жϵ��жϱ�� �� ch32v30x.h ͷ�ļ��в鿴 IRQn_Type ö����
#define PWM_CH1                 (TIM4_PWM_MAP1_CH1_D12)
#define PWM_CH2                 (TIM4_PWM_MAP1_CH2_D13)
#define PWM_CH3                 (TIM4_PWM_MAP1_CH3_D14)
uint8       uart_get_data[64];                                                  // ���ڽ������ݻ�����
uint8       fifo_get_data[64];                                                  // fifo �������������

uint8       get_data = 0;                                                       // �������ݱ���
uint32      fifo_data_count = 0;                                                // fifo ���ݸ���

fifo_struct uart_data_fifo;

#define PIT_CH                   (TIM2_PIT )                                      // ʹ�õ������жϱ�� ����޸� ��Ҫͬ����Ӧ�޸������жϱ���� isr.c �еĵ���
#define PIT_PRIORITY             (TIM2_IRQn)                                      // ��Ӧ�����жϵ��жϱ��

#define LED2                    (B4)

float fgx, fgy, fgz, fax, fay, faz;
float hxj, fyj, hgj, fyj_a0, fyj_a1;  // ��̬��
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

// **************************** �������� ****************************
// loraģ��
// -------------------------------------------------------------------------------------------------------------------
void lora_Init(){
     uart_init(UART_3, 115200, UART3_MAP0_TX_B10, UART3_MAP0_RX_B11);
}

void Float_to_Byte (float f, unsigned char byte[])     // ������ת���ֽ�
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
//        uart_write_byte(UART_3, byte[t_test]);         //�򴮿�1��������
//    }
//
//    sent_byte[0] = 0X00;
//    sent_byte[1] = 0X00;
//    sent_byte[2] = 0X80;
//    sent_byte[3] = 0X7f;
//
//    for (int t_test = 0; t_test < 4; t_test++)
//    {
//        uart_write_byte(UART_3, sent_byte[t_test]);         //�򴮿�1����֡β
//    }
//}

//-------------------------------------------------------------------------------------------------------------------
// �� TIM2 �жϻ�ȡmpu6050���ݡ���������ֵ���������ٶ� ������
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

        fyj=asinf(2*(q0*q2-q1*q3))*5730;//������
        hgj=atan2f((q0*q1+q2*q3),(1-2*(q1*q1+q2*q2)))*5730;
        hxj=atan2f((q0*q3+q1*q2),(1-2*(q2*q2+q3*q3)))*5730;
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_144M);                                         // ��ʼ��оƬʱ�� ����Ƶ��Ϊ 144MHz
    debug_init();                                                          // ��ʼ��Ĭ�� Debug UART

    // �˴���д�û����� ���������ʼ�������
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);                         // ��ʼ�� LED1 ��� Ĭ�ϵ͵�ƽ �������ģʽ

    pit_ms_init(PIT_CH, 100);                                              // ��ʼ�� PIT_CH0 Ϊ�����ж� 100ms ����
    interrupt_set_priority(PIT_PRIORITY, 0);
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // ��ʼ�� fifo ���ػ�����
    lora_Init();
    pwm_init (PWM_CH1, 17000, 7200);     // �޸ĳ�ʼ PWM ռ�ձ�        ��ʵ��̨�Ͽ�����X�����ת
    pwm_init (PWM_CH2, 17000, 7200);     // ��ӦZ����ת
    pwm_init (PWM_CH3, 17000, 7200);      // ��ӦY����ת
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // ���� UART_INDEX �Ľ����ж�
    interrupt_set_priority(UART_PRIORITY, (0<<5) || 1);                         // ���ö�Ӧ UART_INDEX ���ж���ռ���ȼ�0�������ȼ�1

    mpu6050_init();

    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
        if(fifo_data_count != 0)                                                // ��ȡ��������
        {
           fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
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
           // �˴���д������ɨ�����
        }
        system_delay_ms(10);
        if(pit_state)
        {
           PIT2_Run();
           pit_state = 0;                                                      // ��������жϴ�����־λ
        }
        // �˴���д��Ҫѭ��ִ�еĴ���
////    Vofa_JustFloat();
//    system_delay_ms(1);
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
//    get_data = uart_read_byte(UART_INDEX);                                      // �������� while �ȴ�ʽ ���������ж�ʹ��
    uart_query_byte(UART_INDEX, &get_data);                                     // �������� ��ѯʽ �����ݻ᷵�� TRUE û�����ݻ᷵�� FALSE
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // ������д�� fifo ��
}

//-------------------------------------------------------------------------------------------------------------------
// �������     PIT2 �������жϴ����� ����������� PIT2 ��Ӧ�Ķ�ʱ���жϵ��� ��� isr.c
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    pit_state = 1;                                                          // �����жϴ��� ��־λ��λ
}
