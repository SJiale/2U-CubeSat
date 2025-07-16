#include "qmc5883.h"
#include "math.h"
#include "zf_common_headfile.h"

float QMC_X,QMC_Y,QMC_Z; //QMC5883�����������
unsigned char qmcdat[6];
float qmcAngle;
int qmcX,qmcY,qmcZ;

static  soft_iic_info_struct QMC5883_iic_struct;
#define QMC5883_write_register(reg, data)       (soft_iic_write_8bit_register(&QMC5883_iic_struct, (reg), (data)))
#define QMC5883_read_register(reg)              (soft_iic_read_8bit_register(&QMC5883_iic_struct, (reg)))
#define QMC5883_read_registers(reg, data, len)  (soft_iic_read_8bit_registers(&QMC5883_iic_struct, (reg), (data), (len)))


//==========================HMC5883��������=============================
//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ HMC5883 �Ĵ�������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     HMC5883_SB_Read(void)��                                           // ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
//void HMC5883_SB_Read(void)
//{
//    unsigned char dat[6];
//    HMC5883_read_registers(HMC5883_ADDR, dat, 6);
//    X=dat[0] << 8 | dat[1];  //Combine MSB and LSB of X Data output register  �����Чλ
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
// �������     ��ʼ�� HMC5883
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     HMC5883_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void QMC5883_Init(void)
{
     soft_iic_init(&QMC5883_iic_struct, QMC5883_ADDR, QMC5883_SOFT_IIC_DELAY, QMC5883_SCL_PIN, QMC5883_SDA_PIN);
     system_delay_ms(100);                             // �ϵ���ʱ
     QMC5883_write_register(0x09, 0x0d);   // 0x58           //д�Ĵ���A��30Hz�������������ƽ����0
     QMC5883_write_register(0x0b, 0x01);   // 0x60           //д�Ĵ���B������������+-0.88Ga������1370��˹
     QMC5883_write_register(0x20, 0x40);   // 0x00           //д�Ĵ���C�������������
     QMC5883_write_register(0x21, 0x01);   // 0x00           //д�Ĵ���C�������������
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���� HMC5883 ���ؽǶ�
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     HMC5883_Get_Angle();
// ��ע��Ϣ
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

