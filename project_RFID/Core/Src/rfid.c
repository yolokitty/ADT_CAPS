#include "rfid.h"

extern SPI_HandleTypeDef hspi1;

#define RC522_SDA_PORT GPIOC
#define RC522_SDA_PIN GPIO_PIN_8

#define RC522_RST_PORT GPIOC
#define RC522_RST_PIN GPIO_PIN_5

// RC522 통신 상태 전이 단계
typedef enum
{
  TRX_PHASE_IDLE = 0,        // 대기 상태
  TRX_PHASE_SET_BITFRAMING,  // bit framing 설정
  TRX_PHASE_SET_IDLE_CMD,    // command를 idle로 설정
  TRX_PHASE_CLEAR_IRQ,       // irq 플래그 초기화
  TRX_PHASE_FLUSH_FIFO,      // fifo 비우기
  TRX_PHASE_WRITE_FIFO,      // fifo에 송신 데이터 쓰기
  TRX_PHASE_ENABLE_IRQ,      // irq enable
  TRX_PHASE_SET_TRANSCEIVE,  // transceive 명령 설정
  TRX_PHASE_START_SEND,      // 송신 시작 비트 설정
  TRX_PHASE_WAIT_IRQ,        // 응답 대기
  TRX_PHASE_CLEAR_STARTBIT,  // 송신 시작 비트 해제
  TRX_PHASE_CHECK_ERROR,     // 에러 확인
  TRX_PHASE_READ_FIFO_LEVEL, // fifo에 저장된 데이터 길이 확인
  TRX_PHASE_READ_CONTROL,    // control 레지스터 확인
  TRX_PHASE_READ_FIFO,       // fifo 데이터읽기
  TRX_PHASE_FINISH           // 통신 종료
} rc522_trx_phase;

// 정보 저장 구조체
typedef struct
{
  rc522_trx_phase phase;
  uint8_t bitFraming;

  uint8_t sendBuf[32];
  uint8_t sendLen;
  uint8_t sendIdx;

  uint8_t fifoLevelRaw;
  uint8_t fifoReadLen;
  uint8_t readIdx;
  uint8_t lastBits;
} rc522_nb_ctx;

static rc522_nb_ctx trx = {0};

// register
#define CommandReg      0x01
#define ComIEnReg       0x02
#define DivIEnReg       0x03
#define ComIrqReg       0x04
#define DivIrqReg       0x05
#define ErrorReg        0x06
#define Status1Reg      0x07
#define Status2Reg      0x08
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define ControlReg      0x0C
#define BitFramingReg   0x0D
#define ModeReg         0x11
#define TxModeReg       0x12
#define RxModeReg       0x13
#define TxControlReg    0x14
#define TxASKReg        0x15
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D

// RC522 Command
#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C
#define PCD_SOFTRESET   0x0F

// PICC Command
#define PICC_REQA       0x26
#define PICC_ANTICOLL   0x93

static rfid_status rfid_st = RFID_IDLE;
static rc522_st st;
static uint32_t next_scan_tick = 0;
static uint32_t lastTick = 0;

uint8_t ui[4];

static uint8_t anticoll[2] = {PICC_ANTICOLL, 0x20};

extern volatile uint8_t locked;
extern volatile uint8_t accepted;

// 허용된 uid
static const uint8_t AllowedUid[][4] = {{0xD3, 0x6D, 0x80, 0x96}};

// 카드 uid : {0xD3, 0x6D, 0x80, 0x96}

void RC522_SDA_Low()
{
  HAL_GPIO_WritePin(RC522_SDA_PORT, RC522_SDA_PIN, GPIO_PIN_RESET);
}

void RC522_SDA_High()
{
  HAL_GPIO_WritePin(RC522_SDA_PORT, RC522_SDA_PIN, GPIO_PIN_SET);
}

// 특정 비트를 1로
void SetBitMask(uint8_t reg, uint8_t mask)
{
  RC522_WriteReg(reg, RC522_ReadReg(reg) | mask);
}

// 특정 비트를 0으로
void ClearBitMask(uint8_t reg, uint8_t mask)
{
  RC522_WriteReg(reg, RC522_ReadReg(reg) & (~mask));
}

// 레지스터에 값 입력
void RC522_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];
  tx[0] = (uint8_t)((reg << 1) & 0x7E);
  tx[1] = value;

  RC522_SDA_Low();
  (void)HAL_SPI_Transmit(&hspi1, tx, 2, 100);
  RC522_SDA_High();
}

// 동작 중단
void RC522_TransceiveAbort(rc522_st *st)
{
  ClearBitMask(BitFramingReg, 0x80);
  RC522_WriteReg(CommandReg, PCD_IDLE);
  trx.phase = TRX_PHASE_IDLE;
  st->st = FAIL;
}

// 레지스터 값 읽기
uint8_t RC522_ReadReg(uint8_t reg)
{
  uint8_t addr = (uint8_t)(((reg << 1) & 0x7E) | 0x80);
  uint8_t rx = 0;
  uint8_t dummy = 0x00;

  RC522_SDA_Low();

  (void)HAL_SPI_Transmit(&hspi1, &addr, 1, 100);

  (void)HAL_SPI_TransmitReceive(&hspi1, &dummy, &rx, 1, 100);

  RC522_SDA_High();

  return rx;
}

// 초기화
void RC522_Reset()
{
  HAL_GPIO_WritePin(RC522_RST_PORT, RC522_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(RC522_RST_PORT, RC522_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(50);
}

void AntennaOn(void)
{
  uint8_t val = RC522_ReadReg(TxControlReg);
  if((val & 0x03) != 0x03)
  {
    SetBitMask(TxControlReg, 0x03);
  }
}

// 초기화
void RC522_Init(void)
{
  RC522_SDA_High();
  HAL_GPIO_WritePin(RC522_RST_PORT, RC522_RST_PIN, GPIO_PIN_SET);

  RC522_Reset();
  RC522_WriteReg(CommandReg, PCD_SOFTRESET);
  HAL_Delay(50);

  RC522_WriteReg(TModeReg, 0x8D);
  RC522_WriteReg(TPrescalerReg, 0x3E);
  RC522_WriteReg(TReloadRegL, 30);
  RC522_WriteReg(TReloadRegH, 0);

  RC522_WriteReg(TxASKReg, 0x40);
  RC522_WriteReg(ModeReg, 0x3D);

  AntennaOn();
}

// transeive 시작
void RC522_TransceiveStart(rc522_st *st, const uint8_t *send, uint8_t sendLen,
                           uint8_t bitFraming, uint8_t waitIrq, uint32_t timeout)
{
  if(sendLen > sizeof(trx.sendBuf))
  {
    st->st = FAIL;
    return;
  }

  memcpy(trx.sendBuf, send, sendLen);
  trx.sendLen = sendLen;
  trx.sendIdx = 0;
  trx.readIdx = 0;
  trx.fifoLevelRaw = 0;
  trx.fifoReadLen = 0;
  trx.lastBits = 0;
  trx.bitFraming = bitFraming;
  trx.phase = TRX_PHASE_SET_BITFRAMING;

  st->st = RUNNING;
  st->t0 = HAL_GetTick();
  st->timeout = timeout;
  st->waitIrq = waitIrq;
  st->backBytes = 0;
  st->backBits = 0;
}

// 상태전이를 이용하여 non-blocking 구현
status RC522_TransceiveStep(rc522_st *st)
{
  if(st->st != RUNNING)
  {
    return st->st;
  }

  if((HAL_GetTick() - st->t0) > st->timeout)
  {
    RC522_TransceiveAbort(st);
    return st->st;
  }

  switch(trx.phase)
  {
    case TRX_PHASE_SET_BITFRAMING:
      RC522_WriteReg(BitFramingReg, trx.bitFraming);
      trx.phase = TRX_PHASE_SET_IDLE_CMD;
      return RUNNING;

    case TRX_PHASE_SET_IDLE_CMD:
      RC522_WriteReg(CommandReg, PCD_IDLE);
      trx.phase = TRX_PHASE_CLEAR_IRQ;
      return RUNNING;

    case TRX_PHASE_CLEAR_IRQ:
      RC522_WriteReg(ComIrqReg, 0x7F);
      trx.phase = TRX_PHASE_FLUSH_FIFO;
      return RUNNING;

    case TRX_PHASE_FLUSH_FIFO:
      RC522_WriteReg(FIFOLevelReg, 0x80);
      trx.phase = TRX_PHASE_WRITE_FIFO;
      return RUNNING;

    case TRX_PHASE_WRITE_FIFO:
      if(trx.sendIdx < trx.sendLen)
      {
        RC522_WriteReg(FIFODataReg, trx.sendBuf[trx.sendIdx]);
        trx.sendIdx++;
        return RUNNING;
      }
      trx.phase = TRX_PHASE_ENABLE_IRQ;
      return RUNNING;

    case TRX_PHASE_ENABLE_IRQ:
      RC522_WriteReg(ComIEnReg, 0x77 | 0x80);
      trx.phase = TRX_PHASE_SET_TRANSCEIVE;
      return RUNNING;

    case TRX_PHASE_SET_TRANSCEIVE:
      RC522_WriteReg(CommandReg, PCD_TRANSCEIVE);
      trx.phase = TRX_PHASE_START_SEND;
      return RUNNING;

    case TRX_PHASE_START_SEND:
      SetBitMask(BitFramingReg, 0x80);
      trx.phase = TRX_PHASE_WAIT_IRQ;
      return RUNNING;

    case TRX_PHASE_WAIT_IRQ:
    {
      uint8_t n = RC522_ReadReg(ComIrqReg);
      if(n & 0x01)
      {
        RC522_TransceiveAbort(st);
        return st->st;
      }

      if(!(n & st->waitIrq))
      {
        return RUNNING;
      }
      trx.phase = TRX_PHASE_CLEAR_STARTBIT;
      return RUNNING;
    }

    case TRX_PHASE_CLEAR_STARTBIT:
      ClearBitMask(BitFramingReg, 0x80);
      trx.phase = TRX_PHASE_CHECK_ERROR;
      return RUNNING;

    case TRX_PHASE_CHECK_ERROR:
      if(RC522_ReadReg(ErrorReg) & 0x1B)
      {
        RC522_TransceiveAbort(st);
        return st->st;
      }
      trx.phase = TRX_PHASE_READ_FIFO_LEVEL;
      return RUNNING;

    case TRX_PHASE_READ_FIFO_LEVEL:
      trx.fifoLevelRaw = RC522_ReadReg(FIFOLevelReg);
      st->backBytes = trx.fifoLevelRaw;
      trx.phase = TRX_PHASE_READ_CONTROL;
      return RUNNING;

    case TRX_PHASE_READ_CONTROL:
      trx.lastBits = RC522_ReadReg(ControlReg) & 0x07;
      if(trx.lastBits)
      {
        if(trx.fifoLevelRaw == 0)
        {
          st->backBits = trx.lastBits;
        }
        else
        {
          st->backBits = (uint8_t)(((trx.fifoLevelRaw - 1) * 8) + trx.lastBits);
        }
      }
      else
      {
        st->backBits = (uint8_t)(trx.fifoLevelRaw * 8);
      }

      trx.fifoReadLen = trx.fifoLevelRaw;
      if(trx.fifoReadLen > sizeof(st->back))
      {
        trx.fifoReadLen = sizeof(st->back);
      }

      trx.readIdx = 0;
      trx.phase = TRX_PHASE_READ_FIFO;
      return RUNNING;

    case TRX_PHASE_READ_FIFO:
      if(trx.readIdx < trx.fifoReadLen)
      {
        st->back[trx.readIdx] = RC522_ReadReg(FIFODataReg);
        trx.readIdx++;
        return RUNNING;
      }

      trx.phase = TRX_PHASE_FINISH;
      return RUNNING;

    case TRX_PHASE_FINISH:
      RC522_WriteReg(CommandReg, PCD_IDLE);
      trx.phase = TRX_PHASE_IDLE;
      st->st = DONE;
      return st->st;

    case TRX_PHASE_IDLE:
    default:
      return st->st;
  }
}

// 상태에 따라 동작
void RFID_Task()
{
  uint32_t now = HAL_GetTick();

  switch(rfid_st)
  {
    case RFID_IDLE:
    {
      if(now < next_scan_tick)
      {
        break;
      }
      next_scan_tick = now + 50;

      static const uint8_t reqa = PICC_REQA;
      RC522_TransceiveStart(&st, &reqa, 1, 0x07, 0x20, 100);
      rfid_st = RFID_WAIT_REQA;
      break;
    }

    case RFID_WAIT_REQA:
    {
      status s = RC522_TransceiveStep(&st);
      if(s == DONE && st.backBits == 16)
      {
        RC522_TransceiveStart(&st, anticoll, 2, 0x00, 0x20, 100);
        rfid_st = RFID_WAIT_UID;
      }
    }

    case RFID_WAIT_UID:
    {
      status s = RC522_TransceiveStep(&st);
      if(s == RUNNING)
      {
        break;
      }
      if(s == DONE && st.backBits == 40)
      {
        uint8_t bcc = st.back[0] ^ st.back[1] ^ st.back[2] ^ st.back[3];
        if(bcc == st.back[4])
        {
          ui[0] = st.back[0]; ui[1] = st.back[1];
          ui[2] = st.back[2]; ui[3] = st.back[3];
          if(locked)
          {
            if(now - lastTick > 1000)
            {
              lastTick = now;
              if(UID_IsAllowed(ui))
              {
                locked = 0;
                accepted = 1;
              }
              else
              {
                printf("%02X %02X %02X %02X\r\n",ui[0],ui[1],ui[2],ui[3]);
              }
            }
          }
        }
      }
      rfid_st = RFID_IDLE;
      break;
    }
  }
}

// 읽은 uid가 허용 목록에 있는지 확인
bool UID_IsAllowed(const uint8_t ui[4])
{
  for(size_t i = 0; i < (sizeof(AllowedUid)/sizeof(AllowedUid[0])); i++)
  {
    if(memcmp(ui, AllowedUid[i], 4) == 0)
    {
      return true;
    }
  }
  return false;
}
