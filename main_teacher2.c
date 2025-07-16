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
int32 hxj,fyj,hgj;//姿态角

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

    fyj=asinf(2*(q0*q2-q1*q3))*5730;//俯仰角
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
    clock_init(SYSTEM_CLOCK_144M);                                              // 初始化芯片时钟 工作频率为 600MHz
    debug_init();                                                               // 初始化默认 Debug UART

    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);                              // 初始化 LED1 输出 默认低电平 推挽输出模式
    gpio_init(CW_DJ1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ3, GPO, GPIO_LOW, GPO_PUSH_PULL);

    gpio_init(PIN_DJMC1, GPI, 0, GPI_PULL_UP);
    gpio_init(PIN_DJMC2, GPI, 0, GPI_PULL_UP);
    gpio_init(PIN_DJMC3, GPI, 0, GPI_PULL_UP);

    pit_ms_init(PIT_CH, 200);                                                     // 初始化 PIT_CH0 为周期中断 1ms 周期

    interrupt_set_priority(PIT_PRIORITY, 0);                                    // 设置 PIT1 对周期中断的中断优先级为 0

    mpu6050_init();

    pwm_init(PWM_CH1, 10000, 0);                                                // 初始化 PWM 通道 频率 10KHz 初始占空比 0%
    pwm_init(PWM_CH2, 10000, 0);                                                // 初始化 PWM 通道 频率 10KHz 初始占空比 0%
    pwm_init(PWM_CH3, 10000, 0);                                                // 初始化 PWM 通道 频率 10KHz 初始占空比 0%
    j3_pwm=10000;
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
        if(pit_state)
        {
            //
            PIT2_Run();
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
