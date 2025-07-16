// 此文件用来进行电机控制

#include "zf_common_headfile.h"
#include "MahonyAHRS.h"
#include "hmc5883.h"
#include <math.h>
#include <Vofa.h>

// **************************** 代码区域 ****************************
#define UART_INDEX              (UART_3)                        // 默认 UART_3
#define UART_BAUDRATE           (115200)                           // 默认 115200
#define UART_TX_PIN             (UART3_MAP0_TX_B10)                           // 默认 UART3_MAP0_TX_B10
#define UART_RX_PIN             (UART3_MAP0_RX_B11)                           // 默认 UART3_MAP0_RX_B11

#define UART_PRIORITY           (USART3_IRQn)                                   // 对应串口中断的中断编号 在 ch32v30x.h 头文件中查看 IRQn_Type 枚举体

uint8       uart_get_data[64];                                                  // 串口接收数据缓冲区
uint8       fifo_get_data[64];                                                  // fifo 输出读出缓冲区

uint8       get_data = 0;                                                       // 接收数据变量
uint32      fifo_data_count = 0;                                                // fifo 数据个数

fifo_struct uart_data_fifo;

#define PIT_CH                   (TIM2_PIT )                                      // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY             (TIM2_IRQn)                                      // 对应周期中断的中断编号

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
float hxj, fyj, hgj, fyj_a0, fyj_a1;  // 姿态角
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

float Angle_Balance_x;          //X横向角度
float Gyro_Balance_x;     //X横向角加速度
float Angle_Balance_y;          //Y横向角度
float Gyro_Balance_y;     //Y横向角加速度
float Angle_Balance_z;          //Z横向角度
float Gyro_Balance_z;    //Z横向角加速度

int Balance_Pwm_x=0,velocity_Pwm_x=0;  //横向电机PWM分量
int Balance_Pwm_y=0,velocity_Pwm_y=0;  //纵向电机PWM分量
int Balance_Pwm_z=0,velocity_Pwm_z=0;  //Z轴向电机PWM分量
float finalpwmx, finalpwmy, finalpwmz;
uint8 pit_state = 0;

// **************************** 代码区域 ****************************
// lora模块
//-------------------------------------------------------------------------------------------------------------------
void lora_Init(){
     uart_init(UART_3, 115200, UART3_MAP0_TX_B10, UART3_MAP0_RX_B11);
}

void Float_to_Byte (float f, unsigned char byte[])     // 浮点数转四字节
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
        uart_write_byte(UART_3, byte[t_test]);         //向串口1发送数据
    }

    sent_byte[0] = 0X00;
    sent_byte[1] = 0X00;
    sent_byte[2] = 0X80;
    sent_byte[3] = 0X7f;

    for (int t_test = 0; t_test < 4; t_test++)
    {
        uart_write_byte(UART_3, sent_byte[t_test]);         //向串口1发送帧尾
    }
}

//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]

//-------------------------------------------------------------------------------------------------------------------
//  第一种PID 控制方法，角度+速度
//  1.比例控制是引入了回复力
//  2.微分控制是引入了阻尼力
//  3.微分系数与转动惯量有关
//-------------------------------------------------------------------------------------------------------------------

void Yijielvbo_X(float angle_m, float gyro_m)  // 一阶互补滤波  入口参数：加速度、角速度
{
      Angle_Balance_x = 0.06 * angle_m + (1-0.06) * (Angle_Balance_x + gyro_m * 0.01);
}

void Yijielvbo_Y(float angle_m, float gyro_m)  // 一阶互补滤波  入口参数：加速度、角速度
{
      Angle_Balance_y = 0.06 * angle_m + (1-0.06) * (Angle_Balance_y + gyro_m * 0.01);
}

void Yijielvbo_Z(float angle_m, float gyro_m)  // 一阶互补滤波  入口参数：加速度、角速度
{
      Angle_Balance_z = 0.06 * angle_m + (1-0.06) * (Angle_Balance_z + gyro_m * 0.01);
}

void Get_Angle(void)
{
      float Gyro_Y, Gyro_Z, Accel_X, Accel_Z, Accel_Y, Gyro_X;
      Gyro_Y = mpu6050_gyro_y / 16.4;    //读取Y轴陀螺仪并进行量程转换
      Gyro_Z = mpu6050_gyro_z;    //读取Z轴陀螺仪
      Accel_X = mpu6050_acc_x;    //读取X轴加速度计
      Accel_Z = mpu6050_acc_z;    //读取Z轴加速度计
      if(Gyro_Y>32768)  Gyro_Y-=65536;                       //数据类型转换  也可通过short强制类型转换
      if(Gyro_Z>32768)  Gyro_Z-=65536;                       //数据类型转换
      if(Accel_X>32768) Accel_X-=65536;                      //数据类型转换
      if(Accel_Z>32768) Accel_Z-=65536;                      //数据类型转换
      // Gyro_Balance_y=-Gyro_Y;                //更新平衡角速度
      Accel_Y = atan2(Accel_X,Accel_Z) * 180 / PI;  //计算倾角

      Gyro_X = mpu6050_gyro_x / 32.8;       //读取X轴陀螺仪并进行量程转换
      Accel_Y = mpu6050_acc_y;       //读取Y轴加速度计
      if(Gyro_X>32768)  Gyro_X-=65536;   //数据类型转换  也可通过short强制类型转换
      if(Accel_Y>32768) Accel_Y-=65536;  //数据类型转换
      Gyro_Balance_x = -Gyro_X;            //更新平衡角速度
      Accel_X= (atan2(Accel_Z , Accel_Y)) * 180 / PI; //计算倾角
      Accel_Y= (atan2(Accel_X , Accel_Z)) * 180 / PI; //计算倾角
      Accel_Z= (atan2(Accel_X , Accel_Y)) * 180 / PI; //计算倾角

      Yijielvbo_X(Accel_X,Gyro_X);
      Yijielvbo_Y(Accel_Y,Gyro_Y);
      Yijielvbo_Z(Accel_Z,Gyro_Z);
}

//-------------------------------------------------------------------------------------------------------------------
// x方向电机pid
//-------------------------------------------------------------------------------------------------------------------

int balance_x(float Angle,float gyro) //倾角PD控制 入口参数：角度 返回  值：倾角控制PWM
{
     float Balance_KP = 370,Balance_KI = 0,Balance_KD = -2.2;
     float Bias;                                        //倾角偏差
     static float D_Bias,Integral_bias;                 //PID相关变量
     int balance;                                       //PWM返回值
     Bias = Angle - 30;                                   //求出平衡的角度中值 和机械相关
     Integral_bias += Bias;
     if(Integral_bias>30000)Integral_bias=30000;
     if(Integral_bias<-30000)Integral_bias=-30000;
//   D_Bias=gyro;                                         //求出偏差的微分 进行微分控制
     D_Bias *= 0.2;                                            //一阶低通滤波器
     D_Bias += gyro*0.8;                                         //求出偏差的微分 进行微分控制
     balance=Balance_KP*Bias+Balance_KI*Integral_bias+D_Bias*Balance_KD;   //计算倾角控制的电机PWM  PD控制
     return balance;
}

int velocity_x(int encoder)   //位置式PID控制器 入口参数：编码器测量位置信息，目标位置  返回  值：电机PWM
{
     float Speed_KP=65,Speed_KI=0.05,Speed_KD=0;
     static float Pwm,Integral_bias,Last_Bias,Encoder;
     Encoder *= 0.65;                                              //一阶低通滤波器
     Encoder += encoder*0.35;                                    //一阶低通滤波器
     Integral_bias+=Encoder;                                     //求出偏差的积分
     if(Integral_bias>20000)Integral_bias=20000;
     if(Integral_bias<-20000)Integral_bias=-20000;
     Pwm=Speed_KP*Encoder+Speed_KI*Integral_bias+Speed_KD*(Encoder-Last_Bias);       //位置式PID控制器
     Last_Bias=Encoder;                                       //保存上一次偏差
     return Pwm;                                              //增量输出
}

//-------------------------------------------------------------------------------------------------------------------
// y方向电机pid
//-------------------------------------------------------------------------------------------------------------------

int balance_y(float Angle,float gyro) //倾角PD控制 入口参数：角度 返回  值：倾角控制PWM
{
     float Balance_KP = 370,Balance_KI=0,Balance_KD=-2.2;       //kp = 100 kd = -2
     float Bias;                                        //倾角偏差
     static float D_Bias,Integral_bias;                 //PID相关变量
     int balance;                                       //PWM返回值
     Bias = Angle - 60;                                   //求出平衡的角度中值 和机械相关
     Integral_bias+=Bias;
     if(Integral_bias>30000)Integral_bias=30000;
     if(Integral_bias<-30000)Integral_bias=-30000;
//   D_Bias=gyro;                                         //求出偏差的微分 进行微分控制
     D_Bias *= 0.2;                                            //一阶低通滤波器
     D_Bias += gyro*0.8;                                         //求出偏差的微分 进行微分控制
     balance=Balance_KP*Bias+Balance_KI*Integral_bias+D_Bias*Balance_KD;   //计算倾角控制的电机PWM  PD控制
     return balance;
}

int velocity_y(int encoder)   //位置式PID控制器 入口参数：编码器测量位置信息，目标位置  返回  值：电机PWM
{
     float Speed_KP=65,Speed_KI=0.05,Speed_KD=0;
     static float Pwm,Integral_bias,Last_Bias,Encoder;
     Encoder *= 0.65;                                              //一阶低通滤波器
     Encoder += encoder*0.35;                                    //一阶低通滤波器
     Integral_bias+=Encoder;                                     //求出偏差的积分
     if(Integral_bias>20000)Integral_bias=20000;
     if(Integral_bias<-20000)Integral_bias=-20000;
     Pwm=Speed_KP*Encoder+Speed_KI*Integral_bias+Speed_KD*(Encoder-Last_Bias);       //位置式PID控制器
     Last_Bias=Encoder;                                       //保存上一次偏差
     return Pwm;                                              //增量输出
}

//-------------------------------------------------------------------------------------------------------------------
// z方向电机pid
//-------------------------------------------------------------------------------------------------------------------

int balance_z(float Angle,float gyro) //倾角PD控制 入口参数：角度 返回  值：倾角控制PWM
{
     float Balance_KP=370,Balance_KI=0,Balance_KD=-2.2;
     float Bias;                                        //倾角偏差
     static float D_Bias,Integral_bias;                 //PID相关变量
     int balance;                                       //PWM返回值
     Bias = Angle-70;                                   //求出平衡的角度中值 和机械相关
     Integral_bias += Bias;
     if(Integral_bias>30000)Integral_bias=30000;
     if(Integral_bias<-30000)Integral_bias=-30000;
//   D_Bias=gyro;                                         //求出偏差的微分 进行微分控制
     D_Bias *= 0.2;                                            //一阶低通滤波器
     D_Bias += gyro*0.8;                                         //求出偏差的微分 进行微分控制
     balance=Balance_KP*Bias+Balance_KI*Integral_bias+D_Bias*Balance_KD;   //计算倾角控制的电机PWM  PD控制
     return balance;
}

int velocity_z(int encoder)   //位置式PID控制器 入口参数：编码器测量位置信息，目标位置  返回  值：电机PWM
{
     float Speed_KP=65,Speed_KI=0.05,Speed_KD=0;
     static float Pwm,Integral_bias,Last_Bias,Encoder;
     Encoder *= 0.65;                                              //一阶低通滤波器
     Encoder += encoder*0.35;                                    //一阶低通滤波器
     Integral_bias+=Encoder;                                     //求出偏差的积分
     if(Integral_bias>20000)Integral_bias=20000;
     if(Integral_bias<-20000)Integral_bias=-20000;
     Pwm=Speed_KP*Encoder+Speed_KI*Integral_bias+Speed_KD*(Encoder-Last_Bias);       //位置式PID控制器
     Last_Bias=Encoder;                                       //保存上一次偏差
     return Pwm;                                              //增量输出
}

//-------------------------------------------------------------------------------------------------------------------
// PWM限幅
//-------------------------------------------------------------------------------------------------------------------
void Xianfu_Pwm(void)
{
      int Amplitude_x=6900,Amplitude_y=6900,Amplitude_z=6900;  //===PWM满幅是7200 限制在6900
      // 下一步调试中，需要根据卫星质心位置调整每个动量轮PWM限幅大小，对于需要提供更大的动力的轮，限幅更低（低电平有效）
      if(finalpwmx<-Amplitude_x) finalpwmx=-Amplitude_x;
        if(finalpwmx>Amplitude_x)  finalpwmx=Amplitude_x;
      if(finalpwmy<-Amplitude_y) finalpwmy=-Amplitude_y;
        if(finalpwmy>Amplitude_y)  finalpwmy=Amplitude_y;
      if(finalpwmz<-Amplitude_z) finalpwmz=-Amplitude_z;
        if(finalpwmz>Amplitude_z)  finalpwmz=Amplitude_z;
}

//-------------------------------------------------------------------------------------------------------------------
// 在 TIM2 中断获取mpu6050数据、编码器数值、编码器速度 并计算
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

        fyj=asinf(2*(q0*q2-q1*q3))*5730;//俯仰角
        hgj=atan2f((q0*q1+q2*q3),(1-2*(q1*q1+q2*q2)))*5730;
        hxj=atan2f((q0*q3+q1*q2),(1-2*(q2*q2+q3*q3)))*5730;

        Get_Angle();

        Balance_Pwm_x = balance_x(mpu6050_gyro_x, hxj);  //角度 PD 控制
        Balance_Pwm_y = balance_y(mpu6050_gyro_y, fyj);  //角度 D 控制
        Balance_Pwm_z = balance_z(mpu6050_gyro_z, hgj);  //角度 PD 控制

        finalpwmx = Balance_Pwm_x + velocity_Pwm_x;
        finalpwmy = Balance_Pwm_y + velocity_Pwm_y;
        finalpwmz = Balance_Pwm_z + velocity_Pwm_z;

        Xianfu_Pwm();
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_144M);                                         // 初始化芯片时钟 工作频率为 144MHz
    debug_init();                                                          // 初始化默认 Debug UART

    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);                         // 初始化 LED1 输出 默认低电平 推挽输出模式
    gpio_init(CW_DJ1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CW_DJ3, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pit_ms_init(PIT_CH, 100);                                              // 初始化 PIT_CH0 为周期中断 100ms 周期
    interrupt_set_priority(PIT_PRIORITY, 0);
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
    lora_Init();
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_PRIORITY, (0<<5) || 1);                         // 设置对应 UART_INDEX 的中断抢占优先级0，子优先级1

    mpu6050_init();

    pwm_init (PWM_CH1, 17000, 7200);     // 修改初始 PWM 占空比        在实验台上控制j绕X轴的旋转
    pwm_init (PWM_CH2, 17000, 7200);     // 对应Z轴旋转
    pwm_init (PWM_CH3, 17000, 7200);     // 对应Y轴旋转
    pwm_init (PWM_CH4, 17000, 7200);
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        // 此处编写需要循环执行的代码
        fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
        if(fifo_data_count != 0)                                                // 读取到数据了
        {
           fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
           pit_disable(TIM2_PIT);
           // 此处编写立方星扫描代码
           pwm_set_duty (PWM_CH1, 1000);
           pwm_set_duty (PWM_CH2, 7200);
           pwm_set_duty (PWM_CH3, 7200);
           if(fifo_data_count != 0){           // 读取到数据了
               fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
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
                              pwm_set_duty (PWM_CH1, -finalpwmy);  // 如果不加限幅，此处有可能造成最大占空比超出，进入debug     finaly

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
                              pwm_set_duty (PWM_CH2, -finalpwmz);   // finaly   离固定件近的电机，貌似是控制俯仰的

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
           pit_state = 0;                                                      // 清空周期中断触发标志位
        }
        // 此处编写需要循环执行的代码
    Vofa_JustFloat();
    system_delay_ms(1);
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
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // 将数据写入 fifo 中
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT2 的周期中断处理函数 这个函数将在 PIT2 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    pit_state = 1;                                                          // 周期中断触发 标志位置位
}
