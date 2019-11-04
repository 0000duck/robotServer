#include "scsiapi.h"

uint32_t Xil_Out32(uint32_t phyaddr, uint32_t val)
{
	int fd;
	volatile uint8_t *map_base;
	uint32_t base = phyaddr & PAGE_MASK;
	uint32_t pgoffset = phyaddr & (~PAGE_MASK);
 
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
	{
		perror("open /dev/mem:");
		return 1;
	}
 
	map_base = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, base);
	if(map_base == MAP_FAILED)
	{
		perror("mmap:");
		return 2;
	}
	*(volatile uint32_t *)(map_base + pgoffset) = val; 
	close(fd);
	munmap((void *)map_base, PAGE_SIZE);
	
	return 0;
}
 
uint32_t Xil_In32(uint32_t phyaddr)
{
	int fd;
	uint32_t val;
	volatile uint8_t *map_base;
	uint32_t base = phyaddr & PAGE_MASK;
	uint32_t pgoffset = phyaddr & (~PAGE_MASK);
	//open /dev/mem
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
	{
		perror("open /dev/mem:");
	}
	//mmap
	map_base = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, base);
	if(map_base == MAP_FAILED)
	{
		perror("mmap:");
	}
	val = *(volatile uint32_t *)(map_base + pgoffset);
	close(fd);
	munmap((void *)map_base, PAGE_SIZE);
 
	return val;
} 

int fd1;
volatile uint8_t *map_base1;
uint32_t OpenSCSI(uint32_t phyaddr)
{
	volatile uint8_t *map_base;
	uint32_t base = phyaddr & PAGE_MASK;
	uint32_t pgoffset = phyaddr & (~PAGE_MASK);
 
	if((fd1 = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
	{
		perror("open /dev/mem:");
		return 1;
	}
 
	map_base1 = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd1, base);
	if(map_base1 == MAP_FAILED)
	{
		perror("mmap:");
		return 2;
	}
	
	return 0;
}
void SetSCSI_M(uint32_t regaddr, uint32_t val)
{

	*(volatile uint32_t *)(map_base1+regaddr) = val; 
}
 
uint32_t GetSCSI_M(uint32_t regaddr)
{
	uint32_t val;
	val = *(volatile uint32_t *)(map_base1 + regaddr);
	return val;
}
void CloseSCSI()
{
	close(fd1);
	munmap((void *)map_base1, PAGE_SIZE);
}



// SCSI设置API
// 用法  
// scsiaddr 轴编号 SCSI1 SCSI2 ---- SCSI8 或者AXIIO
// regaddr  S开头的功能参数  S_PERIOD  S_PULSENUM -----
// val      设置值32位数 （设置脉冲数S_PULSENUM功能，反转时候需要把数据最高为置1 ，
		 // bit[31]是方向位，0正转，1反转，bit[30:0]为脉冲数）
// 返回值   0：成功  1：打开设备失败一般没有权限原因 2：内存映射失败，地址参数错误或者PL端不匹配

uint32_t SetSCSI(uint32_t scsiaddr, int regaddr , uint32_t val)
{
	return Xil_Out32(scsiaddr + regaddr , val);
}

// SCSI读取API
// 用法  
// scsiaddr 轴编号 SCSI1 SCSI2 ---- SCSI8 或者AXIIO
// regaddr  G开头的功能参数  G_FRAMENUM  G_PULSEPOS -----
// 返回值   PL端返回的数据

uint32_t GetSCSI(uint32_t scsiaddr, int regaddr)
{
	return Xil_In32(scsiaddr + regaddr);
}


// 获取远程状态  
// 返回值   0无状态 1运行 2停止 4回到初始位置
uint32_t GetSTASTORST(void)
{
	uint32_t st;
	OpenSCSI(AXIIO);

	st = GetSCSI_M(G_STASTORST);
	if(st != 0)
	{
		SetSCSI_M(S_STASTORST_CLR,1);
		SetSCSI_M(S_STASTORST_CLR,0);
	}
	CloseSCSI();
	return st;
}

// 设置RS485 自动发送接收切换延时时间一般设置为通讯波特率下一个字节时间长度
// 用法  
// 比如 9600波特率下一个字节长度  (1+8+1)/9600 = 1ms = 100000 (默认)
// 38400波特率下一个字节长度  (1+8+1)/38400 = 260us = 26000
// val      设置值32位数  单位10ns  
// 返回值   0：成功  1：打开设备失败一般没有权限原因 2：内存映射失败，地址参数错误或者PL端不匹配

uint32_t SetRS485DELAY(uint32_t val)
{
	return Xil_Out32(RS485DELAY , val);
}

// 设置RS485 自动发送接收切换延时时间一般设置为通讯波特率下一个字节时间长度
// 用法
// 比如 9600波特率下一个字节长度  (1+8+1)/9600 = 1ms = 100000
// val      设置值32位数  单位10ns
// 返回值   0：成功  1：打开设备失败一般没有权限原因 2：内存映射失败，地址参数错误或者PL端不匹配

uint32_t GetRS485DELAY(void)
{
	return Xil_In32(RS485DELAY);
}

//设置时间片段内发送脉冲数量和分块数量
//scsiaddr SCSI地址
//time 时间单位10ns   
//pulse 要发送的脉冲数
//dir 方向 1为正 0为负
//num 需要分块的数量 
static int s_dir[8] = {0};
void SetSCSI_Pulse(uint32_t scsiaddr, uint32_t time, uint32_t pulse, int dir)
{
    int dirIndex;
    switch(scsiaddr){
    case SCSI1: dirIndex = 0; break;
    case SCSI2: dirIndex = 1; break;
    case SCSI3: dirIndex = 2; break;
    case SCSI4: dirIndex = 3; break;
    case SCSI5: dirIndex = 4; break;
    case SCSI6: dirIndex = 5; break;
    case SCSI7: dirIndex = 6; break;
    case SCSI8: dirIndex = 7; break;
    default:
        break;
    }

    if(dir != s_dir[dirIndex]){
        // 当改变方向时，延时一段时间
        uint32_t dir_pulse_time = 500;
        OpenSCSI(scsiaddr);
        SetSCSI_M(S_PERIOD , dir_pulse_time);
        if(s_dir[dirIndex] == 1)SetSCSI_M(S_PULSENUM , 0);
            else SetSCSI_M(S_PULSENUM , 0|0x80000000);
        CloseSCSI();
        time = time - dir_pulse_time;
    }

    s_dir[dirIndex] = dir;

    if(pulse == 0){
        OpenSCSI(scsiaddr);
        SetSCSI_M(S_PERIOD , time);
        if(dir == 1)SetSCSI_M(S_PULSENUM , 0);
            else SetSCSI_M(S_PULSENUM , 0|0x80000000);
        CloseSCSI();
        return;
    }

    uint32_t div_clk,div_re;
    div_clk = time/pulse;
    div_re = time%pulse;

    if(div_re==0){
        OpenSCSI(scsiaddr);
        SetSCSI_M(S_PERIOD , div_clk);
        if(dir == 1)SetSCSI_M(S_PULSENUM , pulse);
            else SetSCSI_M(S_PULSENUM , pulse|0x80000000);
        CloseSCSI();
    }
    else{
        OpenSCSI(scsiaddr);

        SetSCSI_M(S_PERIOD , div_clk+1);
        if(dir == 1)SetSCSI_M(S_PULSENUM , div_re);
            else SetSCSI_M(S_PULSENUM , div_re|0x80000000);

        SetSCSI_M(S_PERIOD , div_clk);
        if(dir == 1)SetSCSI_M(S_PULSENUM , pulse-div_re);
            else SetSCSI_M(S_PULSENUM , (pulse-div_re)|0x80000000);

        CloseSCSI();
    }
}

// 打开蜂鸣器x毫秒
// 参数time 单位毫秒，最大20秒
void SetBeep(uint32_t time)
{
    OpenSCSI(AXIIO);
    SetSCSI_M(S_BEEP,(time*100000)|0x80000000);
    SetSCSI_M(S_BEEP,0);
    CloseSCSI();
}

// 打开蜂鸣器
void SetBeepON(void)
{
    SetSCSI(AXIIO , S_BEEP , 0x80000000);
}

// 打开蜂鸣器
void SetBeepOFF(void)
{
    SetSCSI(AXIIO , S_BEEP , 0);
}
