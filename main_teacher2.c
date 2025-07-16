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

#define LED1                    (B4)
#define CW_DJ1                  (E2)
#define CW_DJ2                  (E4)
#define CW_DJ3                  (E6)
#define PIN_DJMC1               (E11)
#define PIN_DJMC2               (B7)
#define PIN_DJMC3               (C7)

uint8 pit_state = 0;

#define CHANNEL_NUMBER          (3)

#define PWM_CH1                 (TIM4_PWM_MAP1_CH1_D12)
#define PWM_CH2                 (TIM4_PWM_MAP1_CH2_D13)
#define PWM_CH3                 (TIM4_PWM_MAP1_CH3_D14)

int16 duty = 0;
uint8 channel_index = 0;
pwm_channel_enum channel_list[CHANNEL_NUMBER] = {PWM_CH1, PWM_CH2, PWM_CH3};

float fgx,fgy,fgz,fax,fay,faz;
int32 hxj,fyj,hgj;//��̬��

int16 j1_pwm,j2_pwm,j3_pwm;
int16 j1_pwm_i,j2_pwm_i,j3_pwm_i;
int32 ex,ey,ez,ex_o,ey_o,ez_o;
int16 pid1,pid2,pid3;

int32 encoder_dj1,encoder_dj2,encoder_dj3;

void PIT2_Run(void)
{
    static int cnt=0,cnt1=0;

    static int32 wx=0,wy=0,wz=0;

    cnt++;
    cnt1++;
    if(cnt>=9)
    {
        cnt=0;
        gpio_toggle_level(LED1);

//        if(j1_pwm<=10000)
//        {
//            j1_pwm=j1_pwm+10;
//        }

        printf("%d,%d,%d,%d,%d,%d\r\n",mpu6050_gyro_x,mpu6050_gyro_y,mpu6050_gyro_z,mpu6050_acc_x,mpu6050_acc_y,mpu6050_acc_z);
    }

    //if(cnt1<500) j3_pwm=5000;
    //else         j3_pwm=-5000;
    //if(cnt1>=4)
    {
      //  cnt1=0;
        wy=0;//(fyj+4000)*1;
    }

    mpu6050_get_acc();
    mpu6050_get_gyro();

    float gz_o = mpu6050_gyro_z;
    j2_pwm = -(mpu6050_gyro_z) * 10 + (mpu6050_gyro_z - gz_o) * 100 ;

    if(j2_pwm>0) gpio_set_level(CW_DJ2,0);
    else         gpio_set_level(CW_DJ2,1);

    if(j2_pwm>=10000)  j2_pwm=10000;
    if(j2_pwm<=-10000) j2_pwm=-10000;

    if(j2_pwm>=0) j2_pwm_i=j2_pwm;
    if(j2_pwm<0) j2_pwm_i=-j2_pwm;

    ey=mpu6050_gyro_y-wy;
    j3_pwm=-ey*100 - (ey-ey_o)*0;
    ey_o=ey;
    if(j3_pwm>=0)
    {
        gpio_set_level(CW_DJ3,0);
        //if(j3_pwm<=3500) j3_pwm=3400;
    }
    else
    {
        gpio_set_level(CW_DJ3,1);
        //if(j3_pwm>=-3500) j3_pwm=-3400;
    }
    //j3_pwm=j3_pwm+pid3;
    if(j3_pwm>=10000)  j3_pwm=10000;
    if(j3_pwm<=-10000) j3_pwm=-10000;

    if(j3_pwm>=0) j3_pwm_i=j3_pwm;
    if(j3_pwm<0) j3_pwm_i=-j3_pwm;

    //pwm_set_duty(PWM_CH1, j1_pwm_i);
    //pwm_set_duty(PWM_CH2, j2_pwm_i);
    pwm_set_duty(PWM_CH3, j3_pwm_i);

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
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);                              // ��ʼ�� LED1 ��� Ĭ�ϵ͵�ƽ �������ģʽ
    gpio_init(CW_DJ1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ3, GPO, GPIO_LOW, GPO_PUSH_PULL);

    gpio_init(PIN_DJMC1, GPI, 0, GPI_PULL_UP);
    gpio_init(PIN_DJMC2, GPI, 0, GPI_PULL_UP);
    gpio_init(PIN_DJMC3, GPI, 0, GPI_PULL_UP);

    pit_ms_init(PIT_CH, 200);                                                     // ��ʼ�� PIT_CH0 Ϊ�����ж� 1ms ����

    interrupt_set_priority(PIT_PRIORITY, 0);                                    // ���� PIT1 �������жϵ��ж����ȼ�Ϊ 0

    mpu6050_init();

    pwm_init(PWM_CH1, 10000, 0);                                                // ��ʼ�� PWM ͨ�� Ƶ�� 10KHz ��ʼռ�ձ� 0%
    pwm_init(PWM_CH2, 10000, 0);                                                // ��ʼ�� PWM ͨ�� Ƶ�� 10KHz ��ʼռ�ձ� 0%
    pwm_init(PWM_CH3, 10000, 0);                                                // ��ʼ�� PWM ͨ�� Ƶ�� 10KHz ��ʼռ�ձ� 0%
    j3_pwm=10000;
    // �˴���д�û����� ���������ʼ�������

    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        if(pit_state)
        {
            //
            PIT2_Run();
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
