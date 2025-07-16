#ifndef _hmc5883_h_
#define _hmc5883_h_

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"

//====================================================软件 IIC 驱动====================================================
#define HMC5883_SOFT_IIC_DELAY      (10)                                       // 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
#define HMC5883_SCL_PIN             (B4)                                       // 软件 IIC SCL 引脚 连接 MPU6050 的 SCL 引脚
#define HMC5883_SDA_PIN             (B6)                                       // 软件 IIC SDA 引脚 连接 MPU6050 的 SDA 引脚

#define HMC5883_REG_X_MSB 0x03 //输出X寄存器A中存储测量结果中的MSB（高位数据）
#define HMC5883_REG_X_LSB 0x04 //输出X寄存器B中存储测量结果中的MSB（低位数据）
#define HMC5883_REG_Z_MSB 0x05 //输出Y寄存器A中存储测量结果中的MSB（高位数据）
#define HMC5883_REG_Z_LSB 0x06 //输出Y寄存器B中存储测量结果中的MSB（低位数据）
#define HMC5883_REG_Y_MSB 0x07 //输出Z寄存器A中存储测量结果中的MSB（高位数据）
#define HMC5883_REG_Y_LSB 0x08 //输出Z寄存器B中存储测量结果中的MSB（低位数据）

/* CRA */
#define HMC5883_REG_CRA_MS_MASK      0x03
#define HMC5883_REG_CRA_MS_NORAL     0x00 // default
#define HMC5883_REG_CRA_MS_XYZ_P     0x01
#define HMC5883_REG_CRA_MS_XYZ_N     0x02
#define HMC5883_REG_CRA_MS_REV       0x03

#define HMC5883_REG_CRA_DO_Hz_MASK   0x1C
#define HMC5883_REG_CRA_DO_Hz_0_75   0x00
#define HMC5883_REG_CRA_DO_Hz_1_5    0x04
#define HMC5883_REG_CRA_DO_Hz_3      0x08
#define HMC5883_REG_CRA_DO_Hz_7_5    0x0C
#define HMC5883_REG_CRA_DO_Hz_15     0x10 // default
#define HMC5883_REG_CRA_DO_Hz_30     0x14
#define HMC5883_REG_CRA_DO_Hz_75     0x18
#define HMC5883_REG_CRA_DO_Hz_NONE   0x1C

#define HMC5883_REG_CRA_MA_MASK      0x60
#define HMC5883_REG_CRA_MA_AVG_1     0x00
#define HMC5883_REG_CRA_MA_AVG_2     0x20
#define HMC5883_REG_CRA_MA_AVG_4     0x40
#define HMC5883_REG_CRA_MA_AVG_8     0x60 // default

#define HMC5883_REG_CRA_RUN_MASK     0x80

/* CRB */
#define HMC5883_REG_GN_GA_MASK       0xE0
#define HMC5883_REG_GN_0_88_G1370    0x00
#define HMC5883_REG_GN_1_30_G1090    0x20 // default
#define HMC5883_REG_GN_1_90_G820     0x40
#define HMC5883_REG_GN_2_50_G660     0x60
#define HMC5883_REG_GN_4_00_G440     0x80
#define HMC5883_REG_GN_4_70_G390     0xA0
#define HMC5883_REG_GN_5_60_G330     0xC0
#define HMC5883_REG_GN_8_10_G230     0xE0

/* MODE */
#define HMC5883_REG_MODE_MR_MASK     0x03
#define HMC5883_REG_MODE_MR_CON      0x00
#define HMC5883_REG_MODE_MR_SIG      0x01 // default
#define HMC5883_REG_MODE_MR_IDLE1    0x02
#define HMC5883_REG_MODE_MR_IDLE2    0x03

/* read addr*/
#define HMC5883_ADDR                 0X1E

extern float HMC_X,HMC_Y,HMC_Z; //HMC5883三轴数据输出
extern unsigned char dat[6];
extern float Angle;
extern int X,Y,Z;

void    HMC5883_Get_Angle           (void);
void    HMC5883_SB_Read             (void);
void    HMC5883_Init                (void);
void    Multiple_Read_HMC5883       (void);

#endif
/*-------------------------------------------------------------
IIC地址：0X3C
配置寄存器A：0X00 （用于配置该装置设置的数据输出速率和测量配置）
配置寄存器B: 0X01 （用于设置装置的增益）
模式寄存器： 0X02 （用来设定装置的操作模式）
---------------------------------------------------------------*/
