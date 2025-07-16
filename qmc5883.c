#include "qmc5883.h"
#include "math.h"
#include "zf_common_headfile.h"

float QMC_X,QMC_Y,QMC_Z; //QMC5883三轴数据输出
unsigned char qmcdat[6];
float qmcAngle;
int qmcX,qmcY,qmcZ;

static  soft_iic_info_struct QMC5883_iic_struct;
#define QMC5883_write_register(reg, data)       (soft_iic_write_8bit_register(&QMC5883_iic_struct, (reg), (data)))
#define QMC5883_read_register(reg)              (soft_iic_read_8bit_register(&QMC5883_iic_struct, (reg)))
#define QMC5883_read_registers(reg, data, len)  (soft_iic_read_8bit_registers(&QMC5883_iic_struct, (reg), (data), (len)))


//==========================HMC5883函数定义=============================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 HMC5883 寄存器数据
// 参数说明     void
// 返回参数     void
// 使用示例     HMC5883_SB_Read(void)；                                           // 执行该函数后，直接查看对应的变量即可
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
//void HMC5883_SB_Read(void)
//{
//    unsigned char dat[6];
//    HMC5883_read_registers(HMC5883_ADDR, dat, 6);
//    X=dat[0] << 8 | dat[1];  //Combine MSB and LSB of X Data output register  最高有效位
//    Z=dat[2] << 8 | dat[3];  //Combine MSB and LSB of Z Data output register
//    Y=dat[4] << 8 | dat[5];  //Combine MSB and LSB of Y Data output register
//}

void Multiple_Read_QMC5883(void)
{
    uint8_t buf[6];
    QMC5883_read_registers(0x00, buf, 6);
    qmcX = (int16)(((uint16)buf[0] << 8 | buf[1]));
    qmcY = (int16)(((uint16)buf[2] << 8 | buf[3]));
    qmcZ = (int16)(((uint16)buf[4] << 8 | buf[5]));
    system_delay_ms(5);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 HMC5883
// 参数说明     void
// 返回参数     void
// 使用示例     HMC5883_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void QMC5883_Init(void)
{
     soft_iic_init(&QMC5883_iic_struct, QMC5883_ADDR, QMC5883_SOFT_IIC_DELAY, QMC5883_SCL_PIN, QMC5883_SDA_PIN);
     system_delay_ms(100);                             // 上电延时
     QMC5883_write_register(0x09, 0x0d);   // 0x58           //写寄存器A，30Hz数据输出、采样平均数0
     QMC5883_write_register(0x0b, 0x01);   // 0x60           //写寄存器B，传感器量程+-0.88Ga、增益1370高斯
     QMC5883_write_register(0x20, 0x40);   // 0x00           //写寄存器C，连续数据输出
     QMC5883_write_register(0x21, 0x01);   // 0x00           //写寄存器C，连续数据输出
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算 HMC5883 返回角度
// 参数说明     void
// 返回参数     void
// 使用示例     HMC5883_Get_Angle();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void QMC5883_Get_Angle(void)
{
    u8 i ;
    short Recive_Data[6] ;   //store temperary data
    QMC5883_Init();
    for(i=0; i<6; i++)
    {
        Recive_Data[i] = qmcdat[i];  //get data
    }

    QMC_X = Recive_Data[0]<<8 | Recive_Data[1];//Combine MSB and LSB of X Data output register
    QMC_Z = Recive_Data[2]<<8 | Recive_Data[3];//Combine MSB and LSB of Z Data output register
    QMC_Y = Recive_Data[4]<<8 | Recive_Data[5];//Combine MSB and LSB of Y Data output register

    qmcAngle = atan2((double)QMC_Y,(double)QMC_X) * (180 / 3.14159265) + 180; // angle in degrees
}

