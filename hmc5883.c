#include "hmc5883.h"
#include "math.h"
#include "zf_common_headfile.h"

float HMC_X,HMC_Y,HMC_Z; //HMC5883�����������
unsigned char dat[6];
float Angle;
int X,Y,Z;

static  soft_iic_info_struct HMC5883_iic_struct;
#define HMC5883_write_register(reg, data)       (soft_iic_write_8bit_register(&HMC5883_iic_struct, (reg), (data)))
#define HMC5883_read_register(reg)              (soft_iic_read_8bit_register(&HMC5883_iic_struct, (reg)))
#define HMC5883_read_registers(reg, data, len)  (soft_iic_read_8bit_registers(&HMC5883_iic_struct, (reg), (data), (len)))


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

void Multiple_Read_HMC5883(void)
{
    uint8_t buf[6];
    HMC5883_read_registers(HMC5883_REG_X_MSB, buf, 6);
    X = (int16)(((uint16)buf[0] << 8 | buf[1]));
    Y = (int16)(((uint16)buf[2] << 8 | buf[3]));
    Z = (int16)(((uint16)buf[4] << 8 | buf[5]));
    system_delay_ms(5);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�� HMC5883
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     HMC5883_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void HMC5883_Init(void)
{
     soft_iic_init(&HMC5883_iic_struct, HMC5883_ADDR, HMC5883_SOFT_IIC_DELAY, HMC5883_SCL_PIN, HMC5883_SDA_PIN);
     system_delay_ms(100);                             // �ϵ���ʱ
     HMC5883_write_register(0x00, 0x58);   // 0x58           //д�Ĵ���A��30Hz�������������ƽ����0
     HMC5883_write_register(0x01, 0x60);   // 0x60           //д�Ĵ���B������������+-0.88Ga������1370��˹
     HMC5883_write_register(0x02, 0x00);   // 0x00           //д�Ĵ���C�������������
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���� HMC5883 ���ؽǶ�
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     HMC5883_Get_Angle();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void HMC5883_Get_Angle(void)
{
    u8 i ;
    short Recive_Data[6] ;   //store temperary data
    HMC5883_Init();
    for(i=0; i<6; i++)
    {
        Recive_Data[i] = dat[i];  //get data
    }

    HMC_X = Recive_Data[0]<<8 | Recive_Data[1];//Combine MSB and LSB of X Data output register
    HMC_Z = Recive_Data[2]<<8 | Recive_Data[3];//Combine MSB and LSB of Z Data output register
    HMC_Y = Recive_Data[4]<<8 | Recive_Data[5];//Combine MSB and LSB of Y Data output register

    Angle = atan2((double)HMC_Y,(double)HMC_X) * (180 / 3.14159265) + 180; // angle in degrees
}

