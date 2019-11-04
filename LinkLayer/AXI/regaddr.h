//PL部分
#define AXIIO  0x43C00000
#define SCSI1  0x43C10000
#define SCSI2  0x43C20000
#define SCSI3  0x43C30000
#define SCSI4  0x43C40000
#define SCSI5  0x43C50000
#define SCSI6  0x43C60000
#define SCSI7  0x43C70000
#define SCSI8  0x43C80000

#define RS485DELAY 0x43C90000

//SCSI_1
//设置值32位数 （设置脉冲数S_PULSENUM功能，反转时候需要把数据最高为置1 ，
//bit[31]是方向位，0正转，1反转，bit[30:0]为脉冲数）

#define S_PERIOD       1*4   //脉冲周期  时间单位10ns
#define G_PERIOD       1*4   //读取脉冲周期  时间单位10ns
#define S_PULSENUM     2*4   //脉冲数
#define G_FRAMENUM     3*4   //获取帧数量  最大50个帧
#define S_PULSEPOS     4*4   //设置对应脉冲数
#define G_PULSEPOS     5*4   //获取当前对应脉冲数
#define S_ENCODER_A    6*4   //设置对应编码器数A轴
#define G_ENCODER_A    7*4   //获取当前编码器数
#define S_ENCODER_Z    8*4   //设置对应编码器数Z轴
#define G_ENCODER_Z    9*4   //获取当前编码器数
#define G_SCSISTATE    10*4  //获取轴状态
#define S_ERROR_CLR    11*4  //清除轴错误状态 0x01


//脉冲数定位，编码器定位输出默认初始值为0x7FFFFFFF 即32位数的中间值

//轴状态
//BIT0  急停开关按下
//BIT1  伺服状态未打开
//BIT2  驱动器报错
//BIT3  限位开关触发
//BIT4  HOME回零触发



#define S_SCSI_EN        0    //1:EN 0:DIS
#define G_SCSI_EN_STATE  1*4
#define S_HOME_EN        2*4  // BIT0:SCSI_1  BIT1:SCSI_2  BIT2:SCSI_3 ----- BIT7:SCSI_8
#define G_HOME_STATE     3*4  // 使能状态BIT0:SCSI_1  BIT1:SCSI_2  BIT2:SCSI_3 ----- BIT7:SCSI_8
#define G_DIPSWITCH      4*4
#define G_STASTORST      5*4
#define S_STASTORST_CLR  6*4  // 1清除 0正常
#define G_HOME_FLAG      7*4  // 回零触发 BIT0:SCSI_1  BIT1:SCSI_2  BIT2:SCSI_3 ----- BIT7:SCSI_8
#define S_BEEP                  8*4  // BIT31 1设置  bit30-0  打开时间，单位10ns

//PS部分-----------------------------------------------
#define G_DIN     1
#define G_DOUT    2
#define G_DBOUT   3
#define S_DOUT    4
#define S_DBOUT   5
#define S_AOUT1   6
#define S_AOUT2   7
#define S_AOUT3   8
#define S_AOUT4   9
#define G_AIN1    10
#define G_AIN2    11
#define G_AIN3    12
#define G_AIN4    13
#define G_AOUT1   14
#define G_AOUT2   15
#define G_AOUT3   16
#define G_AOUT4   17
