/*********************************************************************************************************************
* 修改记录
* 日期                                      作者                             备注
* 2022-09-15        大W            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "MahonyAHRS.h"
#include <math.h>
// *************************** 例程硬件连接说明 ***************************
// 核心板正常供电即可 无需额外连接



// **************************** 代码区域 ****************************
#define PIT_CH                  (TIM2_PIT )                                      // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY            (TIM2_IRQn)                                      // 对应周期中断的中断编号

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
float hxj, fyj, hgj, fyj_a0, fyj_a1;  // 姿态角
float mpu_gyro_y0;
float jf_g_add, jf_g[1000];
float roll_output;
float pitch_output;
float yaw_output;

// PID控制器参数
float kp1 = 2;    // 比例系数   50?
float ki1 = 1;    // 积分系数    0.1?
float kd1 = 0.4;    // 微分系数
float kp2 = 2;    // 比例系数
float ki2 = 1;    // 积分系数
float kd2 = 0.4;    // 微分系数
float kp3 = 2;    // 比例系数
float ki3 = 1;    // 积分系数
float kd3 = 0.4;    // 微分系数


// PID控制器状态变量
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
int16 w_zdj, w_ydj; // 左右电机速率值
int16 w_0, w_1; // 速度补偿和差速补偿
int16 pwm_zdj, pwm_ydj, pwm_duty;    // 设置 PWM
int16 PWM1_duty = 2500;
int16 PWM2_duty = 2500;
int16 PWM3_duty = 2500;
int16 j1_pwm,j2_pwm,j3_pwm;
int16 j1_pwm_i,j2_pwm_i,j3_pwm_i;
int32 ex,ey,ez,ex_o,ey_o,ez_o;
int16 pid1,pid2,pid3;

int32 encoder_dj1,encoder_dj2,encoder_dj3;

void pwm2duty()    //  无刷电机 PWM 占空比赋值
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

void w2pwm()   // 限幅 PWM
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

        fyj=asinf(2*(q0*q2-q1*q3))*5730;//俯仰角
        hgj=atan2f((q0*q1+q2*q3),(1-2*(q1*q1+q2*q2)))*5730;
        hxj=atan2f((q0*q3+q1*q2),(1-2*(q2*q2+q3*q3)))*5730;

        encoder_data_zl = encoder_get_count(ENCODER_QUADDEC);               // 获取左轮编码器计数
        encoder_clear_count(ENCODER_QUADDEC);
        encoder_data_yl = -encoder_get_count(ENCODER_QUADDEC1);             // 获取右轮编码器计数
        encoder_clear_count(ENCODER_QUADDEC1);

        pingheng();

        w2pwm();
        pwm2duty();

//    // PID控制器参数
//    float kp = 1.0;    // 比例系数
//    float ki = 0.5;    // 积分系数
//    float kd = 0.2;    // 微分系数
//
//    // PID控制器状态变量
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
////    float error[3] = {-mpu6050_gyro_x, -mpu6050_gyro_y, -mpu6050_gyro_z};         // 误差，分别对应三个轴
////    float integral[3] = {0.0, 0.0, 0.0};      // 积分，分别对应三个轴
////    float derivative[3] = {0.0, 0.0, 0.0};    // 微分，分别对应三个轴
////    float prev_error[3] = {0.0, 0.0, 0.0};    // 上一次误差，分别对应三个轴
//    // 计算误差
////    error[0] = 0 - mpu6050_gyro_x;
////    error[1] = 0 - mpu6050_gyro_y;
////    error[2] = 0 - mpu6050_gyro_z;
//
//    // 计算积分
////    integral[0] += error[0];
////    integral[1] += error[1];
////    integral[2] += error[2];
//
//
//    // 计算微分
////    derivative[0] = error[0] - prev_error[0];
////    derivative[1] = error[1] - prev_error[1];
////    derivative[2] = error[2] - prev_error[2];
//
//    // 计算PID输出
//    roll_output = kp * e1 + ki * i1 + kd * d1;
//    pitch_output = kp * e2 + ki * i2 + kd * d2;
//    yaw_output = kp * e3 + ki * i3 + kd * d3;
//
//    // 更新上一次误差
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
    clock_init(SYSTEM_CLOCK_144M);                                              // 初始化芯片时钟 工作频率为 600MHz
    debug_init();                                                               // 初始化默认 Debug UART

    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);                              // 初始化 LED1 输出 默认低电平 推挽输出模式
    gpio_init(CW_DJ1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ3, GPO, GPIO_LOW, GPO_PUSH_PULL);

//    gpio_init(PIN_DJMC1, GPI, 0, GPI_PULL_UP);
//    gpio_init(PIN_DJMC2, GPI, 0, GPI_PULL_UP);
//    gpio_init(PIN_DJMC3, GPI, 0, GPI_PULL_UP);

    pit_ms_init(PIT_CH, 100);                                                     // 初始化 PIT_CH0 为周期中断 1ms 周期

    interrupt_set_priority(PIT_PRIORITY, 0);                                    // 设置 PIT1 对周期中断的中断优先级为 0

    mpu6050_init();
    encoder_quad_init (ENCODER_QUADDEC, ENCODER_QUADDEC_A, ENCODER_QUADDEC_B);      // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init (ENCODER_QUADDEC1, ENCODER_QUADDEC1_A, ENCODER_QUADDEC1_B);

    pwm_init (PWM_CH1, 17000, 900);    // 修改初始 PWM 占空比
    pwm_init (PWM_CH2, 17000, 900);
    pwm_init (PWM_CH3, 17000, 900);
    pwm_init (PWM_CH4, 17000, 900);
    // j3_pwm=10000;
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
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

            pit_state = 0;                                                      // 清空周期中断触发标志位
        }
        ReadMC();
        // 此处编写需要循环执行的代码
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
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
//
// 问题1：LED 不闪烁
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
