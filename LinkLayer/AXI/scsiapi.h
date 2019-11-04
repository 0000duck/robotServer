#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <unistd.h>
#include <fcntl.h>

#include "regaddr.h"

#define PAGE_SIZE  ((size_t)getpagesize())
#define PAGE_MASK ((uint64_t) (long)~(PAGE_SIZE - 1))

uint32_t Xil_Out32(uint32_t phyaddr, uint32_t val);
uint32_t Xil_In32(uint32_t phyaddr);
uint32_t OpenSCSI(uint32_t phyaddr);
void SetSCSI_M(uint32_t regaddr, uint32_t val);
uint32_t GetSCSI_M(uint32_t regaddr);
void CloseSCSI();

uint32_t SetSCSI(uint32_t scsiaddr, int regaddr , uint32_t val);
uint32_t GetSCSI(uint32_t scsiaddr, int regaddr);
uint32_t GetSTASTORST(void);
uint32_t SetRS485DELAY(uint32_t val);
uint32_t GetRS485DELAY(void);

void SetSCSI_Pulse(uint32_t scsiaddr, uint32_t time, uint32_t pulse, int dir);

void SetBeep(uint32_t time);
void SetBeepON(void);
void SetBeepOFF(void);
