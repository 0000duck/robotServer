#include "stm32api.h"

int stm32fd;
void OpenSTM32(void)
{
    stm32fd = UART0_Open(stm32fd,"/dev/ttyS0"); //打开串口，返回文件描述符
    UART0_Init(stm32fd,115200,0,8,1,'N');
    //tcflush(stm32fd,TCIFLUSH);
}
uint8_t rcv_buf[10]={0x00,0x00,0x00,0x00,0x00};
uint8_t send_buf[10]={0xaa,0x0a,0x00,0x00,0x00};
// addr为G开头寄存器地址 G_DIN G_DOUT ----
// 返回值为16位数
uint16_t GetSTM32(uint8_t addr)
{
    int len;
    send_buf[1] = addr;
    len = UART0_Send(stm32fd,send_buf,4);
    if(len <= 0)printf("send data failed!\n");
    usleep(1000);
    len = UART0_Recv(stm32fd, rcv_buf,4);
#ifdef DEBUGOUT
        rcv_buf[len] = '\0';
        printf("receive data is [%02X][%02X][%02X][%02X]\n",rcv_buf[0],rcv_buf[1],rcv_buf[2],rcv_buf[3]);
#endif

    if((len = 4)&&(rcv_buf[0]==0X55)&&(rcv_buf[1]==send_buf[1]))
    {

        return ((rcv_buf[2]<<8)+rcv_buf[3]);
    }
    else
    {
        printf("cannot receive data\n");
    }
}

// addr为S开头寄存器地址 S_DIN S_DOUT ----
//
// 返回值为16位数
uint16_t SetSTM32(uint8_t addr , uint16_t val)
{
    int len;
    send_buf[1] = addr;
    send_buf[2] = val>>8;
    send_buf[3] = val&0x00ff;
    len = UART0_Send(stm32fd,send_buf,4);
    if(len <= 0)printf("send data failed!\n");
    usleep(1000);
    len = UART0_Recv(stm32fd, rcv_buf,4);
    if((len = 4)&&(rcv_buf[0]==0X55)&&(rcv_buf[1]==send_buf[1])&&(rcv_buf[1]==send_buf[1])&&(rcv_buf[1]==send_buf[1]))
    {
#ifdef DEBUGOUT
        rcv_buf[len] = '\0';
        printf("receive data is [%02X][%02X][%02X][%02X]\n",rcv_buf[0],rcv_buf[1],rcv_buf[2],rcv_buf[3]);
#endif
        return ((rcv_buf[2]<<8)+rcv_buf[3]);
    }
    else
    {
        printf("cannot receive data\n");
    }
}

// 设置运行状态 0停止 1暂停 2运行
// 返回值 0成功 -1失败
// bit7-0 SYS REM PLY TEA SSA PSA RSA ERR
int SetRunStatus(uint8_t val)
{
    uint16_t st,re;
    st = GetSTM32(G_DBOUT);
    switch(val)
    {
        case 0:
            st = st|(1<<3);
            st = st&(~(1<<2));
            st = st&(~(1<<1));
            break;
        case 1:
            st = st|(1<<2);
            st = st&(~(1<<1));
            st = st&(~(1<<3));
            break;
        case 2:
            st = st|(1<<1);
            st = st&(~(1<<2));
            st = st&(~(1<<3));
            break;
        default:break;
    }
    re = SetSTM32(S_DBOUT,st);
    if(re == st)return 0;
        else return -1;
}

// 设置示教再现远程状态 0示教 1再现 2远程
// 返回值 0成功 -1失败
// bit7-0 SYS REM PLY TEA SSA PSA RSA ERR
int SetSJStatus(uint8_t val)
{
    uint16_t st,re;
    st = GetSTM32(G_DBOUT);
    switch(val)
    {
        case 0:
            st = st|(1<<4);
            st = st&(~(1<<5));
            st = st&(~(1<<6));
            break;
        case 1:
            st = st|(1<<5);
            st = st&(~(1<<4));
            st = st&(~(1<<6));
            break;
        case 2:
            st = st|(1<<6);
            st = st&(~(1<<4));
            st = st&(~(1<<5));
            break;
        default:break;
    }
    re = SetSTM32(S_DBOUT,st);
    if(re == st)return 0;
        else return -1;
}
// 设置系统启动状态 1打开 0关闭
// 返回值 0成功 -1失败
// bit7-0 SYS REM PLY TEA SSA PSA RSA ERR
int SetSysStatus(uint8_t val)
{
    uint16_t st,re;
    st = GetSTM32(G_DBOUT);
    switch(val)
    {
        case 0:
            st = st&(~(1<<7));
            break;
        case 1:
            st = st|(1<<7);
            break;
        default:break;
    }
    re = SetSTM32(S_DBOUT,st);
    if(re == st)return 0;
        else return -1;
}

// 设置错误显示 1打开 0关闭
// 返回值 0成功 -1失败
// bit7-0 SYS REM PLY TEA SSA PSA RSA ERR
int SetErrStatus(uint8_t val)
{
    uint16_t st,re;
    st = GetSTM32(G_DBOUT);
    switch(val)
    {
        case 0:
            st = st&(~(1));
            break;
        case 1:
            st = st|(1);
            break;
        default:break;
    }
    re = SetSTM32(S_DBOUT,st);
    if(re == st)return 0;
        else return -1;
}
