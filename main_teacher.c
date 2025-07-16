/*********************************************************************************************************************
* �޸ļ�¼
* ����                                      ����                             ��ע
* 2022-09-15        ��W            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "MahonyAHRS.h"
#include <math.h>
// *************************** ����Ӳ������˵�� ***************************
// ���İ��������缴�� �����������



// **************************** �������� ****************************
#define PIT_CH                  (TIM2_PIT )                                      // ʹ�õ������жϱ�� ����޸� ��Ҫͬ����Ӧ�޸������жϱ���� isr.c �еĵ���
#define PIT_PRIORITY            (TIM2_IRQn)                                      // ��Ӧ�����жϵ��жϱ��

#define LED2                    (B4)
#define CW_DJ1                  (A7)
#define CW_DJ2                  (A9)
#define CW_DJ3                  (A11)
#define PIN_DJMC1               (D12)
#define PIN_DJMC2               (D13)
#define PIN_DJMC3               (D14)

#define ENCODER_QUADDEC                 TIM5_ENCOEDER
#define ENCODER_QUADDEC_A               TIM5_ENCOEDER_MAP0_CH1_A0
#define ENCODER_QUADDEC_B               TIM5_ENCOEDER_MAP0_CH2_A1

#define ENCODER_QUADDEC1                 TIM3_ENCOEDER
#define ENCODER_QUADDEC1_A               TIM3_ENCOEDER_MAP0_CH1_A6
#define ENCODER_QUADDEC1_B               TIM3_ENCOEDER_MAP0_CH2_A7
uint8 pit_state = 0;

#define CHANNEL_NUMBER          (3)

#define PWM_CH1                 (TIM4_PWM_MAP1_CH1_D12)
#define PWM_CH2                 (TIM4_PWM_MAP1_CH2_D13)
#define PWM_CH3                 (TIM4_PWM_MAP1_CH3_D14)
#define PWM_CH4                 TIM4_PWM_MAP1_CH4_D15

int16 duty = 0;
uint8 channel_index = 0;
pwm_channel_enum channel_list[CHANNEL_NUMBER] = {PWM_CH1, PWM_CH2, PWM_CH3};

float fgx, fgy, fgz, fax, fay, faz;
float hxj, fyj, hgj, fyj_a0, fyj_a1;  // ��̬��
float mpu_gyro_y0;
float jf_g_add, jf_g[1000];
float roll_output;
float pitch_output;
float yaw_output;

// PID����������
float kp1 = 2;    // ����ϵ��   50?
float ki1 = 1;    // ����ϵ��    0.1?
float kd1 = 0.4;    // ΢��ϵ��
float kp2 = 2;    // ����ϵ��
float ki2 = 1;    // ����ϵ��
float kd2 = 0.4;    // ΢��ϵ��
float kp3 = 2;    // ����ϵ��
float ki3 = 1;    // ����ϵ��
float kd3 = 0.4;    // ΢��ϵ��


// PID������״̬����
float e1;
float e2;
float e3;
float i1;
float i2;
float i3;
float d1;
float d2;
float d3;
float p1;
float p2;
float p3;
float roll_output;
float pitch_output;
float yaw_output;

int16 encoder_data_zl = 0;
int16 encoder_data_yl = 0;
int16 w_zdj, w_ydj; // ���ҵ������ֵ
int16 w_0, w_1; // �ٶȲ����Ͳ��ٲ���
int16 pwm_zdj, pwm_ydj, pwm_duty;    // ���� PWM
int16 PWM1_duty = 2500;
int16 PWM2_duty = 2500;
int16 PWM3_duty = 2500;
int16 j1_pwm,j2_pwm,j3_pwm;
int16 j1_pwm_i,j2_pwm_i,j3_pwm_i;
int32 ex,ey,ez,ex_o,ey_o,ez_o;
int16 pid1,pid2,pid3;

int32 encoder_dj1,encoder_dj2,encoder_dj3;

void pwm2duty()    //  ��ˢ��� PWM ռ�ձȸ�ֵ
{
    if(pwm_ydj >= 0)
    {
        pwm_duty =  pwm_ydj;
        pwm_set_duty (PWM_CH3, pwm_duty);
        pwm_set_duty (PWM_CH4, 7200);
    }
    else
    {
        pwm_duty = - pwm_ydj;
        pwm_set_duty (PWM_CH4, pwm_duty);
        pwm_set_duty (PWM_CH3, 7200);
    }

    if(pwm_zdj >= 0)
    {
        pwm_duty =  pwm_zdj;
        pwm_set_duty(PWM_CH1, pwm_duty);
        pwm_set_duty(PWM_CH2, 7200);
    }
    else
    {
        pwm_duty = - pwm_zdj;
        pwm_set_duty(PWM_CH2, pwm_duty);
        pwm_set_duty(PWM_CH1, 7200);
    }
}

void w2pwm()   // �޷� PWM
{
    pwm_ydj = pwm_ydj + (-encoder_data_yl - w_ydj);
    pwm_zdj = pwm_zdj + (-encoder_data_zl - w_zdj);

    if (pwm_ydj <= -7200) pwm_ydj = -7200;
    if (pwm_ydj >= 7200) pwm_ydj = 7200;

    if (pwm_zdj <= -7200) pwm_zdj = -7200;
    if (pwm_zdj >= 7200) pwm_zdj = 7200;
}

void pingheng()
{
    static int16 w_a = 0, w_g = 0;
    static int cnt = 0, ii = 0;

    float err_g, err_a;
    static float last_err_g = 0, last_err_a = 0;
    int i = 0;
    cnt ++;

    err_g = mpu6050_gyro_y - mpu_gyro_y0 - w_a;
    jf_g [ii] = mpu6050_gyro_y - mpu_gyro_y0;
    ii ++;
    if (ii > 999) ii = 0;
    jf_g_add = 0;
    for (i = 0; i < 1000; i ++)
    {
        jf_g_add += jf_g [i];
    }
    w_g = err_g * 0.2 - (err_g - last_err_g) * 0.1 + jf_g_add * 0;
    last_err_g = err_g;

    if (w_g >= 100) w_g = 100;
    if (w_g <= -100) w_g = -100;

    if (cnt >= 3)
    {
        cnt = 0;
        err_a = fyj_a0 - fyj;
        w_a = err_a * 40 + (fyj - last_err_a) * 0;
        last_err_a = fyj;
        //w_a=w_a+a_a;
        //if(w_a>=100) w_a=100;
        //if(w_a<=-100) w_a=-100;
    }

    w_zdj = w_g;
    w_ydj = w_g;
}

void PIT2_Run(void)
{
    static int cnt=0,cnt1=0;

//    static int32 wx=0,wy=0,wz=0;

    cnt++;
    cnt1++;
    if(cnt>=9)
    {
        cnt=0;
        gpio_toggle_level(LED2);
    }

    //if(cnt1<500) j3_pwm=5000;
    //else         j3_pwm=-5000;
    //if(cnt1>=4)
//    {
//      //  cnt1=0;
//       wy=0;//(fyj+4000)*1;
//    }

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

        encoder_data_zl = encoder_get_count(ENCODER_QUADDEC);               // ��ȡ���ֱ���������
        encoder_clear_count(ENCODER_QUADDEC);
        encoder_data_yl = -encoder_get_count(ENCODER_QUADDEC1);             // ��ȡ���ֱ���������
        encoder_clear_count(ENCODER_QUADDEC1);

        pingheng();

        w2pwm();
        pwm2duty();

//    // PID����������
//    float kp = 1.0;    // ����ϵ��
//    float ki = 0.5;    // ����ϵ��
//    float kd = 0.2;    // ΢��ϵ��
//
//    // PID������״̬����
//    float e1 = -mpu6050_gyro_x;
//    float e2 = -mpu6050_gyro_y;
//    float e3 = -mpu6050_gyro_z;
//    float i1 = 0;
//    float i2 = 0;
//    float i3 = 0;
//    float d1 = 0;
//    float d2 = 0;
//    float d3 = 0;
//    float p1 = 0;
//    float p2 = 0;
//    float p3 = 0;
//    float roll_output;
//    float pitch_output;
//    float yaw_output;
//
//    d1 = -mpu6050_gyro_x - p1;
//    d2 = -mpu6050_gyro_y - p2;
//    d3 = -mpu6050_gyro_z - p3;
//
//    p1 = e1;
//    p2 = e2;
//    p3 = e3;
//
//    i1 += e1;
//    i2 += e2;
//    i3 += e3;
////    float error[3] = {-mpu6050_gyro_x, -mpu6050_gyro_y, -mpu6050_gyro_z};         // ���ֱ��Ӧ������
////    float integral[3] = {0.0, 0.0, 0.0};      // ���֣��ֱ��Ӧ������
////    float derivative[3] = {0.0, 0.0, 0.0};    // ΢�֣��ֱ��Ӧ������
////    float prev_error[3] = {0.0, 0.0, 0.0};    // ��һ�����ֱ��Ӧ������
//    // �������
////    error[0] = 0 - mpu6050_gyro_x;
////    error[1] = 0 - mpu6050_gyro_y;
////    error[2] = 0 - mpu6050_gyro_z;
//
//    // �������
////    integral[0] += error[0];
////    integral[1] += error[1];
////    integral[2] += error[2];
//
//
//    // ����΢��
////    derivative[0] = error[0] - prev_error[0];
////    derivative[1] = error[1] - prev_error[1];
////    derivative[2] = error[2] - prev_error[2];
//
//    // ����PID���
//    roll_output = kp * e1 + ki * i1 + kd * d1;
//    pitch_output = kp * e2 + ki * i2 + kd * d2;
//    yaw_output = kp * e3 + ki * i3 + kd * d3;
//
//    // ������һ�����
////    prev_error[0] = error[0];
////    prev_error[1] = error[1];
////    prev_error[2] = error[2];
//
//    if(e1 < 0)
//    {
//        gpio_set_level (A7,0);
//    }
//    else
//    {
//        gpio_set_level (A7,1);
//    }
//    if (roll_output >= 9500) roll_output = 9500;
//    if (roll_output <= 2000) roll_output = 2000;
//
//    pwm_set_duty (D12, roll_output);
//
//    if(e2 < 0)
//          {
//              gpio_set_level(A9, 0);
//          }
//          else
//          {
//              gpio_set_level(A9, 1);
//          }
//    if(pitch_output >= 9500) pitch_output = 9500;
//    if(pitch_output <= 2000) pitch_output = 2000;
//
//    pwm_set_duty (D13, pitch_output);
//
//    if(e3 < 0)
//          {
//              gpio_set_level(A11,0);
//          }
//          else
//          {
//              gpio_set_level(A11,1);
//          }
//    if(yaw_output >= 9500) yaw_output = 9500;
//    if(yaw_output <= 2000) yaw_output = 2000;
//
//    pwm_set_duty (D14, yaw_output);

//    if(j2_pwm>0) gpio_set_level(CW_DJ2,0);
//    else         gpio_set_level(CW_DJ2,1);
//
//    if(j2_pwm>=10000)  j2_pwm=10000;
//    if(j2_pwm<=-10000) j2_pwm=-10000;
//
//    if(j2_pwm>=0) j2_pwm_i=j2_pwm;
//    if(j2_pwm<0) j2_pwm_i=-j2_pwm;
//
//    ey=mpu6050_gyro_y-wy;
//    j3_pwm=-ey*100 - (ey-ey_o)*0;
//    ey_o=ey;
//    if(j3_pwm>=0)
//    {
//        gpio_set_level(CW_DJ3,0);
//        //if(j3_pwm<=3500) j3_pwm=3400;
//    }
//    else
//    {
//        gpio_set_level(CW_DJ3,1);
//        //if(j3_pwm>=-3500) j3_pwm=-3400;
//    }
//    //j3_pwm=j3_pwm+pid3;
//    if(j3_pwm>=10000)  j3_pwm=10000;
//    if(j3_pwm<=-10000) j3_pwm=-10000;
//
//    if(j3_pwm>=0) j3_pwm_i=j3_pwm;
//    if(j3_pwm<0) j3_pwm_i=-j3_pwm;
//
//    //pwm_set_duty(PWM_CH1, j1_pwm_i);
//    //pwm_set_duty(PWM_CH2, j2_pwm_i);
//    pwm_set_duty(PWM_CH3, j3_pwm_i);
}

void ReadMC(void)
{
    //static int32_t mc1=0,mc2=0,mc3=0;
    uint8_t bdj1,bdj2,bdj3;
    static uint8_t bdj11=0,bdj22=0,bdj33=0;

    bdj1=gpio_get_level(PIN_DJMC1);
    bdj2=gpio_get_level(PIN_DJMC2);
    bdj3=gpio_get_level(PIN_DJMC3);
    if(bdj11==0 && bdj1==1) encoder_dj1++;
    if(bdj22==0 && bdj2==1) encoder_dj2++;
    if(bdj33==0 && bdj3==1) encoder_dj3++;
    bdj11=bdj1;
    bdj22=bdj2;
    bdj33=bdj3;
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_144M);                                              // ��ʼ��оƬʱ�� ����Ƶ��Ϊ 600MHz
    debug_init();                                                               // ��ʼ��Ĭ�� Debug UART

    // �˴���д�û����� ���������ʼ�������
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);                              // ��ʼ�� LED1 ��� Ĭ�ϵ͵�ƽ �������ģʽ
    gpio_init(CW_DJ1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ3, GPO, GPIO_LOW, GPO_PUSH_PULL);

//    gpio_init(PIN_DJMC1, GPI, 0, GPI_PULL_UP);
//    gpio_init(PIN_DJMC2, GPI, 0, GPI_PULL_UP);
//    gpio_init(PIN_DJMC3, GPI, 0, GPI_PULL_UP);

    pit_ms_init(PIT_CH, 100);                                                     // ��ʼ�� PIT_CH0 Ϊ�����ж� 1ms ����

    interrupt_set_priority(PIT_PRIORITY, 0);                                    // ���� PIT1 �������жϵ��ж����ȼ�Ϊ 0

    mpu6050_init();
    encoder_quad_init (ENCODER_QUADDEC, ENCODER_QUADDEC_A, ENCODER_QUADDEC_B);      // ��ʼ��������ģ�������� �������������ģʽ
    encoder_quad_init (ENCODER_QUADDEC1, ENCODER_QUADDEC1_A, ENCODER_QUADDEC1_B);

    pwm_init (PWM_CH1, 17000, 900);    // �޸ĳ�ʼ PWM ռ�ձ�
    pwm_init (PWM_CH2, 17000, 900);
    pwm_init (PWM_CH3, 17000, 900);
    pwm_init (PWM_CH4, 17000, 900);
    // j3_pwm=10000;
    // �˴���д�û����� ���������ʼ�������

    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        if(pit_state)
        {
                PIT2_Run();

                e1 = -mpu6050_gyro_x;
                e2 = -mpu6050_gyro_y;
                e3 = -mpu6050_gyro_z;

                d1 = -mpu6050_gyro_x - p1;
                d2 = -mpu6050_gyro_y - p2;
                d3 = -mpu6050_gyro_z - p3;

                i1 += e1;
                i2 += e2;
                i3 += e3;

                if (i1>20000) i1 = 20000;
                if (i2>20000) i2 = 20000;
                if (i3>20000) i3 = 20000;

                p1 = e1;
                p2 = e2;
                p3 = e3;

                roll_output = kp1 * e1 + ki1 * i1 + kd1 * d1;
                pitch_output = kp2 * e2 + ki2 * i2 + kd2 * d2;
                yaw_output = kp3 * e3 + ki3 * i3 + kd3 * d3;

                if(e1 < 0)
                   {
                       gpio_set_level (A7,0);
                   }
                   else
                   {
                       gpio_set_level (A7,1);
                   }

                if (roll_output >= 6000) roll_output = 6000;
                if (roll_output <= 1000) roll_output = 1000;

                pwm_set_duty (D12, roll_output);

                 if(e2 < 0)
                         {
                             gpio_set_level(A9, 0);
                         }
                         else
                         {
                             gpio_set_level(A9, 1);
                         }
                   if(pitch_output >= 6000) pitch_output = 6000;
                   if(pitch_output <= 1000) pitch_output = 1000;

                   pwm_set_duty (D13, pitch_output);

                 if(e3 < 0)
                         {
                             gpio_set_level(A11,0);
                         }
                         else
                         {
                             gpio_set_level(A11,1);
                         }
                   if(yaw_output >= 6000) yaw_output = 6000;
                   if(yaw_output <= 1000) yaw_output = 1000;

                   pwm_set_duty (D14, yaw_output);

            pit_state = 0;                                                      // ��������жϴ�����־λ
        }
        ReadMC();
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
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
// **************************** �������� ****************************

// *************************** ���̳�������˵�� ***************************
// ��������ʱ�밴�������������б���
//
// ����1��LED ����˸
//      �鿴�����Ƿ�������¼���Ƿ����ر���ȷ���������¸�λ����
//      ���ñ������Ӧ LED ���ŵ�ѹ�Ƿ�仯��������仯֤������δ���У�����仯֤�� LED ������
