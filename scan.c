// ���ļ��������е������

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

uint8       uart_get_data[64];                                                  // ���ڽ������ݻ�����
uint8       fifo_get_data[64];                                                  // fifo �������������

uint8       get_data = 0;                                                       // �������ݱ���
uint32      fifo_data_count = 0;                                                // fifo ���ݸ���

fifo_struct uart_data_fifo;

#define PIT_CH                   (TIM2_PIT )                                      // ʹ�õ������жϱ�� ����޸� ��Ҫͬ����Ӧ�޸������жϱ���� isr.c �еĵ���
#define PIT_PRIORITY             (TIM2_IRQn)                                      // ��Ӧ�����жϵ��жϱ��

#define LED2                    (B4)
#define CW_DJ1                  (A7)
#define CW_DJ2                  (A9)
#define CW_DJ3                  (A11)
#define PIN_DJMC1               (D12)
#define PIN_DJMC2               (D13)
#define PIN_DJMC3               (D14)

#define PWM_CH1                 (TIM4_PWM_MAP1_CH1_D12)
#define PWM_CH2                 (TIM4_PWM_MAP1_CH2_D13)
#define PWM_CH3                 (TIM4_PWM_MAP1_CH3_D14)
#define PWM_CH4                 (TIM4_PWM_MAP1_CH4_D15)

float fgx, fgy, fgz, fax, fay, faz;
float hxj, fyj, hgj, fyj_a0, fyj_a1;  // ��̬��
float roll_output;
float pitch_output;
float yaw_output;
unsigned char byte[12];
unsigned char xbyte[4];
unsigned char ybyte[4];
unsigned char zbyte[4];
uint16_t Gyro[3];
unsigned char sent_byte[4];
char Gyrox[25];
char Gyroy[25];
char Gyroz[25];

fifo_struct uart_data_fifo;

float Angle_Balance_x;          //X����Ƕ�
float Gyro_Balance_x;     //X����Ǽ��ٶ�
float Angle_Balance_y;          //Y����Ƕ�
float Gyro_Balance_y;     //Y����Ǽ��ٶ�
float Angle_Balance_z;          //Z����Ƕ�
float Gyro_Balance_z;    //Z����Ǽ��ٶ�

int Balance_Pwm_x=0,velocity_Pwm_x=0;  //������PWM����
int Balance_Pwm_y=0,velocity_Pwm_y=0;  //������PWM����
int Balance_Pwm_z=0,velocity_Pwm_z=0;  //Z������PWM����
float finalpwmx, finalpwmy, finalpwmz;
uint8 pit_state = 0;

// **************************** �������� ****************************
// loraģ��
//-------------------------------------------------------------------------------------------------------------------
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

void Vofa_JustFloat ()
{
    Float_to_Byte((fyj/100 + 29 ) * 1.0, xbyte);
    Float_to_Byte((hgj/100 - 15 ) * 1.0, ybyte);
    Float_to_Byte((hxj/100 + 25 ) * 1.0, zbyte);

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
        uart_write_byte(UART_3, byte[t_test]);         //�򴮿�1��������
    }

    sent_byte[0] = 0X00;
    sent_byte[1] = 0X00;
    sent_byte[2] = 0X80;
    sent_byte[3] = 0X7f;

    for (int t_test = 0; t_test < 4; t_test++)
    {
        uart_write_byte(UART_3, sent_byte[t_test]);         //�򴮿�1����֡β
    }
}

//pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]

//-------------------------------------------------------------------------------------------------------------------
//  ��һ��PID ���Ʒ������Ƕ�+�ٶ�
//  1.���������������˻ظ���
//  2.΢�ֿ�����������������
//  3.΢��ϵ����ת�������й�
//-------------------------------------------------------------------------------------------------------------------

void Yijielvbo_X(float angle_m, float gyro_m)  // һ�׻����˲�  ��ڲ��������ٶȡ����ٶ�
{
      Angle_Balance_x = 0.06 * angle_m + (1-0.06) * (Angle_Balance_x + gyro_m * 0.01);
}

void Yijielvbo_Y(float angle_m, float gyro_m)  // һ�׻����˲�  ��ڲ��������ٶȡ����ٶ�
{
      Angle_Balance_y = 0.06 * angle_m + (1-0.06) * (Angle_Balance_y + gyro_m * 0.01);
}

void Yijielvbo_Z(float angle_m, float gyro_m)  // һ�׻����˲�  ��ڲ��������ٶȡ����ٶ�
{
      Angle_Balance_z = 0.06 * angle_m + (1-0.06) * (Angle_Balance_z + gyro_m * 0.01);
}

void Get_Angle(void)
{
      float Gyro_Y, Gyro_Z, Accel_X, Accel_Z, Accel_Y, Gyro_X;
      Gyro_Y = mpu6050_gyro_y / 16.4;    //��ȡY�������ǲ���������ת��
      Gyro_Z = mpu6050_gyro_z;    //��ȡZ��������
      Accel_X = mpu6050_acc_x;    //��ȡX����ٶȼ�
      Accel_Z = mpu6050_acc_z;    //��ȡZ����ٶȼ�
      if(Gyro_Y>32768)  Gyro_Y-=65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
      if(Gyro_Z>32768)  Gyro_Z-=65536;                       //��������ת��
      if(Accel_X>32768) Accel_X-=65536;                      //��������ת��
      if(Accel_Z>32768) Accel_Z-=65536;                      //��������ת��
      // Gyro_Balance_y=-Gyro_Y;                //����ƽ����ٶ�
      Accel_Y = atan2(Accel_X,Accel_Z) * 180 / PI;  //�������

      Gyro_X = mpu6050_gyro_x / 32.8;       //��ȡX�������ǲ���������ת��
      Accel_Y = mpu6050_acc_y;       //��ȡY����ٶȼ�
      if(Gyro_X>32768)  Gyro_X-=65536;   //��������ת��  Ҳ��ͨ��shortǿ������ת��
      if(Accel_Y>32768) Accel_Y-=65536;  //��������ת��
      Gyro_Balance_x = -Gyro_X;            //����ƽ����ٶ�
      Accel_X= (atan2(Accel_Z , Accel_Y)) * 180 / PI; //�������
      Accel_Y= (atan2(Accel_X , Accel_Z)) * 180 / PI; //�������
      Accel_Z= (atan2(Accel_X , Accel_Y)) * 180 / PI; //�������

      Yijielvbo_X(Accel_X,Gyro_X);
      Yijielvbo_Y(Accel_Y,Gyro_Y);
      Yijielvbo_Z(Accel_Z,Gyro_Z);
}

//-------------------------------------------------------------------------------------------------------------------
// x������pid
//-------------------------------------------------------------------------------------------------------------------

int balance_x(float Angle,float gyro) //���PD���� ��ڲ������Ƕ� ����  ֵ����ǿ���PWM
{
     float Balance_KP = 370,Balance_KI = 0,Balance_KD = -2.2;
     float Bias;                                        //���ƫ��
     static float D_Bias,Integral_bias;                 //PID��ر���
     int balance;                                       //PWM����ֵ
     Bias = Angle - 30;                                   //���ƽ��ĽǶ���ֵ �ͻ�е���
     Integral_bias += Bias;
     if(Integral_bias>30000)Integral_bias=30000;
     if(Integral_bias<-30000)Integral_bias=-30000;
//   D_Bias=gyro;                                         //���ƫ���΢�� ����΢�ֿ���
     D_Bias *= 0.2;                                            //һ�׵�ͨ�˲���
     D_Bias += gyro*0.8;                                         //���ƫ���΢�� ����΢�ֿ���
     balance=Balance_KP*Bias+Balance_KI*Integral_bias+D_Bias*Balance_KD;   //������ǿ��Ƶĵ��PWM  PD����
     return balance;
}

int velocity_x(int encoder)   //λ��ʽPID������ ��ڲ���������������λ����Ϣ��Ŀ��λ��  ����  ֵ�����PWM
{
     float Speed_KP=65,Speed_KI=0.05,Speed_KD=0;
     static float Pwm,Integral_bias,Last_Bias,Encoder;
     Encoder *= 0.65;                                              //һ�׵�ͨ�˲���
     Encoder += encoder*0.35;                                    //һ�׵�ͨ�˲���
     Integral_bias+=Encoder;                                     //���ƫ��Ļ���
     if(Integral_bias>20000)Integral_bias=20000;
     if(Integral_bias<-20000)Integral_bias=-20000;
     Pwm=Speed_KP*Encoder+Speed_KI*Integral_bias+Speed_KD*(Encoder-Last_Bias);       //λ��ʽPID������
     Last_Bias=Encoder;                                       //������һ��ƫ��
     return Pwm;                                              //�������
}

//-------------------------------------------------------------------------------------------------------------------
// y������pid
//-------------------------------------------------------------------------------------------------------------------

int balance_y(float Angle,float gyro) //���PD���� ��ڲ������Ƕ� ����  ֵ����ǿ���PWM
{
     float Balance_KP = 370,Balance_KI=0,Balance_KD=-2.2;       //kp = 100 kd = -2
     float Bias;                                        //���ƫ��
     static float D_Bias,Integral_bias;                 //PID��ر���
     int balance;                                       //PWM����ֵ
     Bias = Angle - 60;                                   //���ƽ��ĽǶ���ֵ �ͻ�е���
     Integral_bias+=Bias;
     if(Integral_bias>30000)Integral_bias=30000;
     if(Integral_bias<-30000)Integral_bias=-30000;
//   D_Bias=gyro;                                         //���ƫ���΢�� ����΢�ֿ���
     D_Bias *= 0.2;                                            //һ�׵�ͨ�˲���
     D_Bias += gyro*0.8;                                         //���ƫ���΢�� ����΢�ֿ���
     balance=Balance_KP*Bias+Balance_KI*Integral_bias+D_Bias*Balance_KD;   //������ǿ��Ƶĵ��PWM  PD����
     return balance;
}

int velocity_y(int encoder)   //λ��ʽPID������ ��ڲ���������������λ����Ϣ��Ŀ��λ��  ����  ֵ�����PWM
{
     float Speed_KP=65,Speed_KI=0.05,Speed_KD=0;
     static float Pwm,Integral_bias,Last_Bias,Encoder;
     Encoder *= 0.65;                                              //һ�׵�ͨ�˲���
     Encoder += encoder*0.35;                                    //һ�׵�ͨ�˲���
     Integral_bias+=Encoder;                                     //���ƫ��Ļ���
     if(Integral_bias>20000)Integral_bias=20000;
     if(Integral_bias<-20000)Integral_bias=-20000;
     Pwm=Speed_KP*Encoder+Speed_KI*Integral_bias+Speed_KD*(Encoder-Last_Bias);       //λ��ʽPID������
     Last_Bias=Encoder;                                       //������һ��ƫ��
     return Pwm;                                              //�������
}

//-------------------------------------------------------------------------------------------------------------------
// z������pid
//-------------------------------------------------------------------------------------------------------------------

int balance_z(float Angle,float gyro) //���PD���� ��ڲ������Ƕ� ����  ֵ����ǿ���PWM
{
     float Balance_KP=370,Balance_KI=0,Balance_KD=-2.2;
     float Bias;                                        //���ƫ��
     static float D_Bias,Integral_bias;                 //PID��ر���
     int balance;                                       //PWM����ֵ
     Bias = Angle-70;                                   //���ƽ��ĽǶ���ֵ �ͻ�е���
     Integral_bias += Bias;
     if(Integral_bias>30000)Integral_bias=30000;
     if(Integral_bias<-30000)Integral_bias=-30000;
//   D_Bias=gyro;                                         //���ƫ���΢�� ����΢�ֿ���
     D_Bias *= 0.2;                                            //һ�׵�ͨ�˲���
     D_Bias += gyro*0.8;                                         //���ƫ���΢�� ����΢�ֿ���
     balance=Balance_KP*Bias+Balance_KI*Integral_bias+D_Bias*Balance_KD;   //������ǿ��Ƶĵ��PWM  PD����
     return balance;
}

int velocity_z(int encoder)   //λ��ʽPID������ ��ڲ���������������λ����Ϣ��Ŀ��λ��  ����  ֵ�����PWM
{
     float Speed_KP=65,Speed_KI=0.05,Speed_KD=0;
     static float Pwm,Integral_bias,Last_Bias,Encoder;
     Encoder *= 0.65;                                              //һ�׵�ͨ�˲���
     Encoder += encoder*0.35;                                    //һ�׵�ͨ�˲���
     Integral_bias+=Encoder;                                     //���ƫ��Ļ���
     if(Integral_bias>20000)Integral_bias=20000;
     if(Integral_bias<-20000)Integral_bias=-20000;
     Pwm=Speed_KP*Encoder+Speed_KI*Integral_bias+Speed_KD*(Encoder-Last_Bias);       //λ��ʽPID������
     Last_Bias=Encoder;                                       //������һ��ƫ��
     return Pwm;                                              //�������
}

//-------------------------------------------------------------------------------------------------------------------
// PWM�޷�
//-------------------------------------------------------------------------------------------------------------------
void Xianfu_Pwm(void)
{
      int Amplitude_x=6900,Amplitude_y=6900,Amplitude_z=6900;  //===PWM������7200 ������6900
      // ��һ�������У���Ҫ������������λ�õ���ÿ��������PWM�޷���С��������Ҫ�ṩ����Ķ������֣��޷����ͣ��͵�ƽ��Ч��
      if(finalpwmx<-Amplitude_x) finalpwmx=-Amplitude_x;
        if(finalpwmx>Amplitude_x)  finalpwmx=Amplitude_x;
      if(finalpwmy<-Amplitude_y) finalpwmy=-Amplitude_y;
        if(finalpwmy>Amplitude_y)  finalpwmy=Amplitude_y;
      if(finalpwmz<-Amplitude_z) finalpwmz=-Amplitude_z;
        if(finalpwmz>Amplitude_z)  finalpwmz=Amplitude_z;
}

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

        Get_Angle();

        Balance_Pwm_x = balance_x(mpu6050_gyro_x, hxj);  //�Ƕ� PD ����
        Balance_Pwm_y = balance_y(mpu6050_gyro_y, fyj);  //�Ƕ� D ����
        Balance_Pwm_z = balance_z(mpu6050_gyro_z, hgj);  //�Ƕ� PD ����

        finalpwmx = Balance_Pwm_x + velocity_Pwm_x;
        finalpwmy = Balance_Pwm_y + velocity_Pwm_y;
        finalpwmz = Balance_Pwm_z + velocity_Pwm_z;

        Xianfu_Pwm();
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_144M);                                         // ��ʼ��оƬʱ�� ����Ƶ��Ϊ 144MHz
    debug_init();                                                          // ��ʼ��Ĭ�� Debug UART

    // �˴���д�û����� ���������ʼ�������
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);                         // ��ʼ�� LED1 ��� Ĭ�ϵ͵�ƽ �������ģʽ
    gpio_init(CW_DJ1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ3, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pit_ms_init(PIT_CH, 100);                                              // ��ʼ�� PIT_CH0 Ϊ�����ж� 100ms ����
    interrupt_set_priority(PIT_PRIORITY, 0);
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // ��ʼ�� fifo ���ػ�����
    lora_Init();
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // ���� UART_INDEX �Ľ����ж�
    interrupt_set_priority(UART_PRIORITY, (0<<5) || 1);                         // ���ö�Ӧ UART_INDEX ���ж���ռ���ȼ�0�������ȼ�1

    mpu6050_init();

    pwm_init (PWM_CH1, 17000, 7200);     // �޸ĳ�ʼ PWM ռ�ձ�        ��ʵ��̨�Ͽ���j��X�����ת
    pwm_init (PWM_CH2, 17000, 7200);     // ��ӦZ����ת
    pwm_init (PWM_CH3, 17000, 7200);     // ��ӦY����ת
    pwm_init (PWM_CH4, 17000, 7200);
    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        fifo_data_count = fifo_used(&uart_data_fifo);                           // �鿴 fifo �Ƿ�������
        if(fifo_data_count != 0)                                                // ��ȡ��������
        {
           fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
           pit_disable(TIM2_PIT);
           // �˴���д������ɨ�����
           pwm_set_duty (PWM_CH1, 1000);
           pwm_set_duty (PWM_CH2, 7200);
           pwm_set_duty (PWM_CH3, 7200);
           if(fifo_data_count != 0){           // ��ȡ��������
               fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
               pit_enable(TIM2_PIT);
           }
        }
        system_delay_ms(10);
        if(pit_state)
        {
           PIT2_Run();
           if(finalpwmy < 0)
                                   {
                                       gpio_set_level (A7,0);
                                   }
                                   else
                                   {
                                       gpio_set_level (A7,1);
                                   }
                      //        if (roll_output > 3000) roll_output = 3000;
                      //        if (roll_output < 1000) roll_output = 1000;
                              pwm_set_duty (PWM_CH1, -finalpwmy);  // ��������޷����˴��п���������ռ�ձȳ���������debug     finaly

                              if(finalpwmz < 0)
                                         {
                                             gpio_set_level(A9, 0);
                                         }
                                         else
                                         {
                                             gpio_set_level(A9, 1);
                                         }
                      //        if (pitch_output > 3000) pitch_output = 3000;
                      //        if (pitch_output < 1000) pitch_output = 1000;
                              pwm_set_duty (PWM_CH2, -finalpwmz);   // finaly   ��̶������ĵ����ò���ǿ��Ƹ�����

                              if(finalpwmx < 0)
                                         {
                                             gpio_set_level(A11,0);
                                         }
                                         else
                                         {
                                             gpio_set_level(A11,1);
                                         }
                      //        if (yaw_output > 3000) yaw_output = 3000;
                      //        if (yaw_output < 1000) yaw_output = 1000;
                              pwm_set_duty (PWM_CH3, -finalpwmx);     //  1111     finalx
           pit_state = 0;                                                      // ��������жϴ�����־λ
        }
        // �˴���д��Ҫѭ��ִ�еĴ���
    Vofa_JustFloat();
    system_delay_ms(1);
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
