#include "UART.h"
#include "regaddr.h"

void OpenSTM32(void);
uint16_t GetSTM32(uint8_t addr);
uint16_t SetSTM32(uint8_t addr , uint16_t val);
int SetRunStatus(uint8_t val);
int SetSJStatus(uint8_t val);
int SetSysStatus(uint8_t val);
int SetErrStatus(uint8_t val);
