#ifndef INC_RFID_H_
#define INC_RFID_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

typedef enum
{
  IDLE = 0,
  RUNNING,
  DONE,
  FAIL
}status;

typedef struct
{
  status st;
  uint32_t t0;
  uint32_t timeout;

  uint8_t waitIrq;
  uint8_t back[16];
  uint8_t backBytes;
  uint8_t backBits;
}rc522_st;

typedef enum
{
  RFID_IDLE = 0,
  RFID_WAIT_REQA,
  RFID_WAIT_UID
}rfid_status;
void RC522_TransceiveAbort(rc522_st *st);
void RC522_SDA_Low();
void RC522_SDA_High();
void SetBitMask(uint8_t reg, uint8_t mask);
void ClearBitMask(uint8_t reg, uint8_t mask);
void RC522_Reset();
uint8_t RC522_ReadReg(uint8_t reg);
void RC522_WriteReg(uint8_t reg, uint8_t value);
void AntennaOn(void);
void RC522_Init();
void RC522_TransceiveStart(rc522_st *st, const uint8_t *send, uint8_t sendLen,
                           uint8_t bitFraming, uint8_t waitIrq, uint32_t timeout);
status RC522_TransceiveStep(rc522_st *st);
void RFID_Task();
bool UID_IsAllowed(const uint8_t ui[4]);

#endif /* INC_RFID_H_ */
