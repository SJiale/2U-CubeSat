#ifndef _VOFA
#define _VOFA

typedef union     //定义结构体
{
        float fdata;
        unsigned long ldata;
} FloatLongType;

extern float temp;

void Float_to_Byte (float f, unsigned char byte[]);

void Vofa_JustFloat (void);
void Vofa_JustFloat_Wireless (void);
void Vofa_PS2_Test (void);
#endif
