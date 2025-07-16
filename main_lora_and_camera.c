// 此文件用来测试lora模块是否按照PC端指令控制摄像头拍照

#include "zf_common_headfile.h"
uint8_t Serial_RxPacket2[10000];
// **************************** 代码区域 ****************************
//-------------------------------------------------------------------------------------------------------------------
// 摄像头模块
//-------------------------------------------------------------------------------------------------------------------
void VC0706_Init(){
    uart_init(UART_2, 115200, UART2_MAP0_TX_A2, UART2_MAP0_RX_A3);
}

void VC0706_TakePicture(void) {
    // 第一步停止当前帧更新
    uint8_t Serial_TxPacket1[5];
    Serial_TxPacket1[0] = 0x56;
    Serial_TxPacket1[1] = 0x00;
    Serial_TxPacket1[2] = 0x36;
    Serial_TxPacket1[3] = 0x01;
    Serial_TxPacket1[4] = 0x00;
    uart_tx_interrupt(UART_2, ENABLE);
    uart_write_buffer(UART_2, Serial_TxPacket1, 5);
    uart_tx_interrupt(UART_2, DISABLE);
    // 第二步获取图片长度，图片长度为 4Bytes
    uint8_t Serial_TxPacket2[5];
    uint8_t Serial_RxPacket1[9];
    uint8_t PictureLength[4];
    Serial_TxPacket2[0] = 0x56;
    Serial_TxPacket2[1] = 0x00;
    Serial_TxPacket2[2] = 0x34;
    Serial_TxPacket2[3] = 0x01;
    Serial_TxPacket2[4] = 0x00;
    uart_tx_interrupt(UART_2, ENABLE);
    uart_write_buffer(UART_2, Serial_TxPacket2, 5);
    uart_tx_interrupt(UART_2, DISABLE);
    uart_rx_interrupt(UART_2, ENABLE);
    uart_query_byte(UART_2, Serial_RxPacket1);
    uart_rx_interrupt(UART_2, DISABLE);
    PictureLength[0] = Serial_RxPacket1[5];
    PictureLength[1] = Serial_RxPacket1[6];
    PictureLength[2] = Serial_RxPacket1[7];
    PictureLength[3] = Serial_RxPacket1[8];
    // 第三步 获取图片
    uint8_t Serial_TxPacket3[16];
    uint8_t Serial_RxPacket2[10000];
    Serial_TxPacket3[0] = 0x56;
    Serial_TxPacket3[1] = 0x00;
    Serial_TxPacket3[2] = 0x32;
    Serial_TxPacket3[3] = 0x0C;
    Serial_TxPacket3[4] = 0x00;
    Serial_TxPacket3[5] = 0x0A;
    Serial_TxPacket3[6] = 0x00;
    Serial_TxPacket3[7] = 0x00;
    Serial_TxPacket3[8] = 0x00;
    Serial_TxPacket3[9] = 0x00;
    Serial_TxPacket3[10] = PictureLength[0];
    Serial_TxPacket3[11] =  PictureLength[1];
    Serial_TxPacket3[12] = PictureLength[2];
    Serial_TxPacket3[13] = PictureLength[3];
    Serial_TxPacket3[14] = 0x00;
    Serial_TxPacket3[15] = 0xFF;
    uart_tx_interrupt(UART_2, ENABLE);
    uart_write_buffer(UART_2, Serial_TxPacket3, 16);
    uart_tx_interrupt(UART_2, DISABLE);
    uart_rx_interrupt(UART_2, ENABLE);
    uart_query_byte(UART_2, Serial_RxPacket2);
    uart_rx_interrupt(UART_2, DISABLE);
    // 第四步 恢复帧更新
    uint8_t Serial_TxPacket4[5];
    Serial_TxPacket4[0] = 0x56;
    Serial_TxPacket4[1] = 0x00;
    Serial_TxPacket4[2] = 0x36;
    Serial_TxPacket4[3] = 0x01;
    Serial_TxPacket4[4] = 0x02;
    uart_tx_interrupt(UART_2, ENABLE);
    uart_write_buffer(UART_2, Serial_TxPacket4, 5);
    uart_tx_interrupt(UART_2, DISABLE);
}

//-------------------------------------------------------------------------------------------------------------------
// lora模块
//-------------------------------------------------------------------------------------------------------------------

void lora_Init(){
     uart_init(UART_3, 9600, UART3_MAP0_TX_B10, UART3_MAP0_RX_B11);
}

void controllora(){
    uint8_t yes[1];
    uart_rx_interrupt(UART_2, ENABLE);
    uart_query_byte(UART_3, yes);
    uart_rx_interrupt(UART_2,DISABLE);
    if (yes[0] == 1) {
        VC0706_Init();
        VC0706_TakePicture();
    }
}

void lorasendbackdata(){
     uint8_t imagedata[10000];
     int i;
     for (i = 0; i < 10000; ++i) {
        imagedata [i] = Serial_RxPacket2 [i+5];
     }
     uart_tx_interrupt(UART_3, ENABLE);
     uart_write_buffer(UART_3, imagedata, 10000);
     uart_tx_interrupt(UART_3, DISABLE);
}

void PIT2_Run(void){

}

int main (void){
    lora_Init();
    controllora();
    lorasendbackdata();
    VC0706_Init();
    VC0706_TakePicture();
    system_delay_ms(1000);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
//void pit_handler (void)
//{
//    pit_state = 1;                                                              // 周期中断触发 标志位置位
//}
