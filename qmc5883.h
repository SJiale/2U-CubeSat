#ifndef _qmc5883_h_
#define _qmc5883_h_

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"

//====================================================软件 IIC 驱动====================================================
#define QMC5883_SOFT_IIC_DELAY      (10)                                       // 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
#define QMC5883_SCL_PIN             (B4)                                       // 软件 IIC SCL 引脚 连接 MPU6050 的 SCL 引脚
#define QMC5883_SDA_PIN             (B6)                                       // 软件 IIC SDA 引脚 连接 MPU6050 的 SDA 引脚

#define QMC5883_REG_DATA       0x00
#define QMC5883_REG_OUT_X_L    0x00
#define QMC5883_REG_OUT_X_M    0x01
#define QMC5883_REG_OUT_Y_L    0x02
#define QMC5883_REG_OUT_Y_M    0x03
#define QMC5883_REG_OUT_Z_L    0x04
#define QMC5883_REG_OUT_Z_M    0x05

#define QMC5883_REG_STATUS     0x06
#define QMC5883_DRDY_BIT0      //0: no new data, 1: new data is ready
#define QMC5883_OVL_BIT1       //0: normal,      1: data overflow
#define QMC5883_DOR_BIT2       //0: normal,      1: data skipped for reading

#define QMC5883_REG_TEMP_OUT_L 0x07
#define QMC5883_REG_TEMP_OUT_H 0x08

#define QMC5883_REG_CTRL1            0x09
#define QMC5883_CMD_MODE_STANDBY     0x00  //mode
#define QMC5883_CMD_MODE_CON         0x01
#define QMC5883_CMD_ODR_10HZ         0x00  //Output Data Rate
#define QMC5883_CMD_ODR_50HZ         0x04
#define QMC5883_CMD_ODR_100HZ        0x08
#define QMC5883_CMD_ODR_200HZ        0x0C
#define QMC5883_CMD_RNG_2G           0x00  //Full Scale
#define QMC5883_CMD_RNG_8G           0x10
#define QMC5883_CMD_OSR_512          0x00  //Over Sample Ratio
#define QMC5883_CMD_OSR_256          0x40
#define QMC5883_CMD_OSR_128          0x80
#define QMC5883_CMD_OSR_64           0xC0

#define QMC5883_REG_CTRL2            0x0A
#define QMC5883_CMD_INT_ENABLE       0x00
#define QMC5883_CMD_INT_DISABLE      0x01
#define QMC5883_CMD_ROL_PNT_ENABLE   0x40  //pointer roll-over function,only 0x00-0x06 address
#define QMC5883_CMD_ROL_PNT_DISABLE  0x00
#define QMC5883_CMD_SOFT_RST_ENABLE  0x80
#define QMC5883_CMD_SOFT_RST_DISABLE 0x00

#define QMC5883_REG_SET_RESET  0x0B
#define QMC5883_CMD_SET_RESET  0x01

#define QMC5883_REG_PRODUCTID  0x0D     //chip id :0xFF

#define QMC5883_ADDR   0x1A

extern float QMC_X,QMC_Y,QMC_Z;  //HMC5883三轴数据输出
extern unsigned char qmcdat[6];
extern float qmcAngle;
extern int qmcX,qmcY,qmcZ;

void    QMC5883_Get_Angle           (void);
void    QMC5883_SB_Read             (void);
void    QMC5883_Init                (void);
void    Multiple_Read_QMC5883       (void);


#endif

/*-------------------------------------------------------------
IIC地址：0X1a
---------------------------------------------------------------*/
