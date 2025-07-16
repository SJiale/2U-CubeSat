// ���ļ��������е������

#include "zf_common_headfile.h"
#include "MahonyAHRS.h"
#include "hmc5883.h"
#include <math.h>
#include <Vofa.h>

// **************************** �������� ****************************
#define PIT_CH                   (TIM2_PIT )                     // ʹ�õ������жϱ�� ����޸� ��Ҫͬ����Ӧ�޸������жϱ���� isr.c �еĵ���
#define PIT_PRIORITY             (TIM2_IRQn)                     // ��Ӧ�����жϵ��жϱ��

#define UART_INDEX              (UART_3)                         // Ĭ�� UART_1
#define UART_BAUDRATE           (115200)                           // Ĭ�� 9600
#define UART_TX_PIN             (UART3_MAP0_TX_B10)                // Ĭ�� UART_1_MAP0_TX_A9
#define UART_RX_PIN             (UART3_MAP0_RX_B11)               // Ĭ�� UART1_MAP0_RX_A10

#define UART_PRIORITY           (USART3_IRQn)

#define LED2                    (B4)
#define CW_DJ1                  (E2)
#define CW_DJ2                  (E4)
#define CW_DJ3                  (E6)
#define PIN_DJMC1               (D12)
#define PIN_DJMC2               (D13)
#define PIN_DJMC3               (D14)

#define ENCODER_RESOLUTION      (100)    //������һȦ������������  100�߱�����

#define ENCODER_QUADDEC1                 TIM5_ENCOEDER
#define ENCODER_QUADDEC1_A               TIM5_ENCOEDER_MAP0_CH1_A0
#define ENCODER_QUADDEC1_B               TIM5_ENCOEDER_MAP0_CH2_A1

#define ENCODER_QUADDEC2                 TIM3_ENCOEDER
#define ENCODER_QUADDEC2_A               TIM3_ENCOEDER_MAP0_CH1_A6
#define ENCODER_QUADDEC2_B               TIM3_ENCOEDER_MAP0_CH2_A7

#define ENCODER_QUADDEC3                 TIM10_ENCOEDER
#define ENCODER_QUADDEC3_A               TIM10_ENCOEDER_MAP0_CH2_B9
#define ENCODER_QUADDEC3_B               TIM10_ENCOEDER_MAP0_CH1_B8

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
int16 duty = 6900;
unsigned char sent_byte[4];
char Gyrox[25];
char Gyroy[25];
char Gyroz[25];
uint8       uart_get_data[64];                                                  // ���ڽ������ݻ�����
uint8       fifo_get_data[64];                                                  // fifo �������������
uint8       get_data = 0;                                                       // �������ݱ���
uint32      fifo_data_count = 0;                                                // fifo ���ݸ���
fifo_struct uart_data_fifo;

//extern int X,Y,Z;

// PID����������
float kp1 = 50;    // ����ϵ��   50?              kp1 ki1 kd1 �������� DJ1
float ki1 = 0.1;    // ����ϵ��    0.1?
float kd1 = 2;    // ΢��ϵ��
float kp2 = 50;    // ����ϵ��        kp2 ki2 kd2 ��������DJ2
float ki2 = 0.1;    // ����ϵ��
float kd2 = 2;    // ΢��ϵ��
float kp3 = 50;    // ����ϵ��         kp3 ki3 kd3 ��������DJ3
float ki3 = 0.1;    // ����ϵ��
float kd3 = 2;    // ΢��ϵ��

//����pid��ʽ
//float Balance_KP_x=350, Balance_KI_x=0, Balance_KD_x=-2;
//float Speed_KP_x=50, Speed_KI_x=0.05, Speed_KD_x=0;

// PID������״̬����
float e1;   // p
float e2;
float e3;
float i1;   // i
float i2;
float i3;
float d1;   // d
float d2;
float d3;
float p1;   // ��һ�����
float p2;
float p3;

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

int16 encoder_data_xl = 0;
int16 encoder_data_yl = 0;
int16 encoder_data_zl = 0;
int16 motorspeedx = 0;
int16 motorspeedy = 0;
int16 motorspeedz = 0;
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

//    Float_to_Byte(mpu6050_gyro_y * 1.0, byte);
//    for (int h_test = 5; h_test < 9; h_test++)
//    {
//        uart_write_byte(UART_3, byte[h_test]);         //�򴮿�1��������
//    }
//
//    Float_to_Byte(mpu6050_gyro_z * 1.0, byte);
//    for (int k_test = 9; k_test < 12; k_test++)
//    {
//        uart_write_byte(UART_3, byte[k_test]);         //�򴮿�1��������
//    }

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
     float Balance_KP = 270,Balance_KI = 0,Balance_KD = -2.2;
     float Bias;                                        //���ƫ��
     static float D_Bias,Integral_bias;                 //PID��ر���
     int balance;                                       //PWM����ֵ
     Bias = Angle - 90;                                   //���ƽ��ĽǶ���ֵ �ͻ�е���
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
     float Balance_KP = 270,Balance_KI=0,Balance_KD=-2.2;       //kp = 100 kd = -2
     float Bias;                                        //���ƫ��
     static float D_Bias,Integral_bias;                 //PID��ر���
     int balance;                                       //PWM����ֵ
     Bias = Angle - 90;                                   //���ƽ��ĽǶ���ֵ �ͻ�е���
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
     float Balance_KP = 270,Balance_KI=0,Balance_KD=-2.2;    // ע�⣡������ԭ����KP��370
     float Bias;                                        //���ƫ��
     static float D_Bias,Integral_bias;                 //PID��ر���
     int balance;                                       //PWM����ֵ
     Bias = Angle-90;                                   //���ƽ��ĽǶ���ֵ �ͻ�е���
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
//  �ڶ���PID ���Ʒ������Ƕ�
//  1.���������������˻ظ���
//  2.΢�ֿ�����������������
//  3.΢��ϵ����ת�������й�
//-------------------------------------------------------------------------------------------------------------------

void PID_Postionx ()
{
    float iIncpid1;
    int get_point=0;//�ڻ�����ֵ
    float in_pid1, out_pid1;
    e1 = 0 - fyj;
    d1 = -fyj - p1;
    i1 += e1;
    if (i1>20000) i1 = 20000;
    if (i1<-20000) i1 = -20000;
    p1 = e1;
    // kp=10 ki=0.1 kd=2
    iIncpid1 = kp1 * e1 + ki1 * i1 + kd1 * d1;
    out_pid1 = iIncpid1;
    in_pid1 = out_pid1-get_point;//�õ�ƫ��ֵ
    roll_output = in_pid1;
}

void PID_Postiony ()
{
    float iIncpid2;
    int get_point=0;//�ڻ�����ֵ
    float in_pid2, out_pid2;
    e2 = 0 - hgj;
    d2 = -hgj - p2;
    i2 += e2;
    if (i2>20000) i2 = 20000;
    if (i2<-20000) i2 = -20000;
    p2 = e2;
    // kp=10 ki=0.1 kd=2
    iIncpid2 = kp2 * e2 + ki2 * i2 + kd2 * d2;
    out_pid2 = iIncpid2;
    in_pid2 = out_pid2-get_point;//�õ�ƫ��ֵ
    pitch_output = in_pid2;
}

void PID_Postionz ()
{
    float iIncpid3;
    int get_point = 0;//�ڻ�����ֵ
    float in_pid3, out_pid3;
    e3 = 0 - hxj;
    d3 = -hxj - p3;
    i3 += e3;
    if (i2>20000) i2 = 20000;
    if (i2<-20000) i2 = -20000;
    p3 = e3;
    // kp=10 ki=0.1 kd=2
    yaw_output = kp3 * e3 + ki3 * i3 + kd3 * d3;
    iIncpid3 = kp3 * e3 + ki3 * i3 + kd3 * d3;
    out_pid3 = iIncpid3;
    in_pid3 = out_pid3-get_point;//�õ�ƫ��ֵ
    yaw_output = in_pid3;
}

void Xianfu_Pwm(void)
{
      int Amplitude_x=6900,Amplitude_y=6900,Amplitude_z=6900;  //===PWM������7200 ������6900
      int LOWERAmplitude_x=2000,LOWERAmplitude_y=2000,LOWERAmplitude_z=2000;  //===PWM������7200 ������6900
      // ��һ�������У���Ҫ������������λ�õ���ÿ��������PWM�޷���С��������Ҫ�ṩ����Ķ������֣��޷����ͣ��͵�ƽ��Ч��
      if(finalpwmx<-Amplitude_x) finalpwmx=-Amplitude_x;
        if(finalpwmx>Amplitude_x)  finalpwmx=Amplitude_x;
      if(finalpwmy<-Amplitude_y) finalpwmy=-Amplitude_y;
        if(finalpwmy>Amplitude_y)  finalpwmy=Amplitude_y;
      if(finalpwmz<-Amplitude_z) finalpwmz=-Amplitude_z;
        if(finalpwmz>Amplitude_z)  finalpwmz=Amplitude_z;
}

void PWMSET ()
{
    if(finalpwmy < 0)
                        {
                            gpio_set_level (E2,0);
                        }
                        else
                        {
                            gpio_set_level (E2,1);
                        }
                   pwm_set_duty (PWM_CH1, -finalpwmy);  // ��������޷����˴��п���������ռ�ձȳ���������debug     finaly

                   if(finalpwmz < 0)
                              {
                                  gpio_set_level(E4, 0);
                              }
                              else
                              {
                                  gpio_set_level(E4, 1);
                              }
                   pwm_set_duty (PWM_CH2, -finalpwmz);   // finaly   ��̶������ĵ��������������������   ��������finalzЧ�����ã�������finalxЧ������

                   if(finalpwmx < 0)
                              {
                                  gpio_set_level(E6,0);
                              }
                              else
                              {
                                  gpio_set_level(E6,1);
                              }
                   pwm_set_duty (PWM_CH3, -finalpwmx);     //  1111     finalx
           pit_state = 0;                                                      // ��������жϴ�����־λ
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

        encoder_data_xl = encoder_get_count(ENCODER_QUADDEC1);               // ��ȡX���ֱ���������
        encoder_clear_count(ENCODER_QUADDEC1);
        encoder_data_yl = -encoder_get_count(ENCODER_QUADDEC2);             // ��ȡy���ֱ���������
        encoder_clear_count(ENCODER_QUADDEC2);
        encoder_data_zl = -encoder_get_count(ENCODER_QUADDEC3);             // ��ȡy���ֱ���������
        encoder_clear_count(ENCODER_QUADDEC3);

        // ����������
        motorspeedx = encoder_data_xl / ENCODER_RESOLUTION * 10;     // ����ʱ��ϵ�����ж����ھ��� ����100ms���ж� ϵ��Ϊ10
        motorspeedy = encoder_data_yl / ENCODER_RESOLUTION * 10;
        motorspeedz = encoder_data_zl / ENCODER_RESOLUTION * 10;

        Get_Angle();

        Balance_Pwm_x = balance_x(mpu6050_gyro_x, hxj/100);  //�Ƕ� PD ����
        velocity_Pwm_x = velocity_x(motorspeedx); //�ٶ� PI ����
        Balance_Pwm_y = balance_y(mpu6050_gyro_y, fyj/100);  //�Ƕ� D ����
        velocity_Pwm_y = velocity_y(motorspeedy); //�ٶ� PI ����
        Balance_Pwm_z = balance_z(mpu6050_gyro_z, hgj/100);  //�Ƕ� PD ����
        velocity_Pwm_z = velocity_z(motorspeedz); //�ٶ� PI ����

        finalpwmx = Balance_Pwm_x + velocity_Pwm_x;
        finalpwmy = Balance_Pwm_y + velocity_Pwm_y;
        finalpwmz = Balance_Pwm_z + velocity_Pwm_z;

        Xianfu_Pwm();
        //Multiple_Read_HMC5883();

        //sprintf(Gyrox, " %d" , mpu6050_gyro_x + 30);
        //sprintf(Gyroy, " %d" , mpu6050_gyro_y - 18);
        //sprintf(Gyroz, " %d" , mpu6050_gyro_z + 25);
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_144M);                                              // ��ʼ��оƬʱ�� ����Ƶ��Ϊ 144MHz
    debug_init();                                                               // ��ʼ��Ĭ�� Debug UART

    // �˴���д�û����� ���������ʼ�������
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);                              // ��ʼ�� LED1 ��� Ĭ�ϵ͵�ƽ �������ģʽ
    gpio_init(CW_DJ1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ3, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pit_ms_init(PIT_CH, 100);                                                     // ��ʼ�� PIT_CH2 Ϊ�����ж� 100ms ����
//    pit_ms_init(TIM3_PIT, 100);
//    pit_ms_init(TIM5_PIT, 100);
//    pit_ms_init(TIM10_PIT, 100);
    interrupt_set_priority(PIT_PRIORITY, 0);
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // ��ʼ�� fifo ���ػ�����
    lora_Init();
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                  // ���� UART_INDEX �Ľ����ж�
    interrupt_set_priority(UART_PRIORITY, (0<<5) || 1);        // ���� UART �������жϵ��ж����ȼ�
    //HMC5883_Init();
    mpu6050_init();
    //gnss_init(TAU1201);
    encoder_quad_init (ENCODER_QUADDEC1, ENCODER_QUADDEC1_A, ENCODER_QUADDEC1_B);      // ��ʼ��������ģ�������� �������������ģʽ
    encoder_quad_init (ENCODER_QUADDEC2, ENCODER_QUADDEC2_A, ENCODER_QUADDEC2_B);
    encoder_quad_init (ENCODER_QUADDEC3, ENCODER_QUADDEC3_A, ENCODER_QUADDEC3_B);

    pwm_init (PWM_CH1, 17000, 7200);     // �޸ĳ�ʼ PWM ռ�ձ�        ��ʵ��̨�Ͽ�����X�����ת
    pwm_init (PWM_CH2, 17000, 7200);     // ��ӦZ����ת
    pwm_init (PWM_CH3, 17000, 7200);      // ��ӦY����ת
    pwm_init (PWM_CH4, 17000, 7200);
    // �˴���д�û����� ���������ʼ�������
    // double Angle_XY=0,Angle_XZ=0,Angle_YZ=0;
    while(1)
    {
      if(pit_state)
      {
        PIT2_Run();
        PWMSET();
      }
               fifo_data_count = fifo_used(&uart_data_fifo);                  // �鿴 fifo �Ƿ�������
               if(fifo_data_count != 0)                                       // ��ȡ��������
               {
                  fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
                  uart_write_string(UART_INDEX, "UART get data:");
                  uart_write_buffer(UART_INDEX, fifo_get_data, fifo_data_count);
                  if (fifo_data_count == 4) {
                      pit_disable(TIM2_PIT);
                      pwm_set_duty (PWM_CH1, 7200);
//                      for (duty = 6900; duty > 500; --duty) {
//                        pwm_set_duty(PWM_CH2, duty);
//                        system_delay_ms(100);
////                        if (duty == 500) {
////                            duty = 6900;
//                        }
//                      }
                      int16 i = 0;
                      for (i = 0; i < 10; ++i) {
                          gpio_set_level(E4,1);
                          pwm_set_duty (PWM_CH2, 4000);     // �˵��������������ɨ��
                          system_delay_ms(100);
                      }
                      int16 k = 0;
                      for (k = 0; k < 10; ++k) {
                          gpio_set_level(E4,0);
                          pwm_set_duty (PWM_CH2, 4000);
                          system_delay_ms(100);
                      }
                      pwm_set_duty(PWM_CH2, 4000);
                      pwm_set_duty (PWM_CH3, 7200);
           //         Vofa_JustFloat1();
                  }
                  if (fifo_data_count == 5) {
                      pit_enable(TIM2_PIT);
                      pwm_set_duty (PWM_CH1, 6000);
                      pwm_set_duty (PWM_CH2, 6000);
                      pwm_set_duty (PWM_CH3, 6000);
                  }
                  if (fifo_data_count == 2) {
                  while(1){
                            pit_enable(TIM2_PIT);
                            PIT2_Run();
                            Vofa_JustFloat();
                            system_delay_ms(1);
                          }
                  }
                  // �˴���д������ɨ�����
               }
        // �˴���д��Ҫѭ��ִ�еĴ���
//      gnss_data_parse();
        //---------��ʾX��
//        Angle_XY= atan2((double)Y,(double)X) * (180 / 3.14159265) + 180; //����XYƽ��Ƕ�
//        Angle_XY*=10;
//
//        Angle_XZ= atan2((double)Z,(double)X) * (180 / 3.14159265) + 180; //����XZƽ��Ƕ�
//        Angle_XZ*=10;
//
//        Angle_YZ= atan2((double)Z,(double)Y) * (180 / 3.14159265) + 180; //����YZƽ��Ƕ�
//        Angle_YZ*=10;
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
// �������     UART_INDEX �Ľ����жϴ����� ����������� UART_INDEX ��Ӧ���жϵ��� ��� isr.c
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
//void uart_rx_interrupt_handler (void)
//{
//     debug_interrupr_handler();
//     gnss_uart_callback();
//}

//-------------------------------------------------------------------------------------------------------------------
// �������     PIT2 �������жϴ����� ����������� PIT2 ��Ӧ�Ķ�ʱ���жϵ��� ��� isr.c
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
//    encoder_data_xl = encoder_get_count(ENCODER_QUADDEC1);                  // ��ȡ����������
//    encoder_data_yl = encoder_get_count(ENCODER_QUADDEC2);                  // ��ȡ����������
//    encoder_data_zl = encoder_get_count(ENCODER_QUADDEC3);                  // ��ȡ����������
//    encoder_clear_count(ENCODER_QUADDEC1);                                  // ��ձ���������
//    encoder_clear_count(ENCODER_QUADDEC2);                                  // ��ձ���������
//    encoder_clear_count(ENCODER_QUADDEC3);                                  // ��ձ���������
    pit_state = 1;                                                          // �����жϴ��� ��־λ��λ
}
