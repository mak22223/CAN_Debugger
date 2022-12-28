/*
 * MCP2515.c
 *
 *  Created on: Dec 28, 2022
 *      Author: mak22
 */

#include "MCP2515/MCP2515.h"

/* ------------- MCP2515 define declaration -------------- */

#define RXBNCTRL_RXM_MASK (0x60UL)
#define RXBNCTRL_RXM (5)

/* ----------- MCP2515 private enum declaration ------------ */

typedef enum {
  MCP_OK = 0U,
  MCP_TIMEOUT
} McpError;

typedef enum {
  MCP_RESET          = 0xC0,
  MCP_READ           = 0x03,
  MCP_READ_RX_BUFFER = 0x90,
  MCP_WRITE          = 0x02,
  MCP_LOAD_TX_BUFFER = 0x40,
  MCP_RTS            = 0x80,
  MCP_READ_STATUS    = 0xA0,
  MCP_RX_STATUS      = 0xB0,
  MCP_BIT_MODIFY     = 0x05
} McpCommand;

typedef enum {
  MCP_RXF0SIDH = 0x00,
  MCP_RXF0SIDL = 0x01,
  MCP_RXF0EID8 = 0x02,
  MCP_RXF0EID0 = 0x03,
  MCP_RXF1SIDH = 0x04,
  MCP_RXF1SIDL = 0x05,
  MCP_RXF1EID8 = 0x06,
  MCP_RXF1EID0 = 0x07,
  MCP_RXF2SIDH = 0x08,
  MCP_RXF2SIDL = 0x09,
  MCP_RXF2EID8 = 0x0A,
  MCP_RXF2EID0 = 0x0B,
  MCP_BFPCTRL  = 0x0C,
  MCP_TXRTSCTRL= 0x0D,
  MCP_CANSTAT  = 0x0E,
  MCP_CANCTRL  = 0x0F,
  MCP_RXF3SIDH = 0x10,
  MCP_RXF3SIDL = 0x11,
  MCP_RXF3EID8 = 0x12,
  MCP_RXF3EID0 = 0x13,
  MCP_RXF4SIDH = 0x14,
  MCP_RXF4SIDL = 0x15,
  MCP_RXF4EID8 = 0x16,
  MCP_RXF4EID0 = 0x17,
  MCP_RXF5SIDH = 0x18,
  MCP_RXF5SIDL = 0x19,
  MCP_RXF5EID8 = 0x1A,
  MCP_RXF5EID0 = 0x1B,
  MCP_TEC      = 0x1C,
  MCP_REC      = 0x1D,
  MCP_RXM0SIDH = 0x20,
  MCP_RXM0SIDL = 0x21,
  MCP_RXM0EID8 = 0x22,
  MCP_RXM0EID0 = 0x23,
  MCP_RXM1SIDH = 0x24,
  MCP_RXM1SIDL = 0x25,
  MCP_RXM1EID8 = 0x26,
  MCP_RXM1EID0 = 0x27,
  MCP_CNF3     = 0x28,
  MCP_CNF2     = 0x29,
  MCP_CNF1     = 0x2A,
  MCP_CANINTE  = 0x2B,
  MCP_CANINTF  = 0x2C,
  MCP_EFLG     = 0x2D,
  MCP_TXB0CTRL = 0x30,
  MCP_TXB0SIDH = 0x31,
  MCP_TXB0SIDL = 0x32,
  MCP_TXB0EID8 = 0x33,
  MCP_TXB0EID0 = 0x34,
  MCP_TXB0DLC  = 0x35,
  MCP_TXB0DATA = 0x36,
  MCP_TXB1CTRL = 0x40,
  MCP_TXB1SIDH = 0x41,
  MCP_TXB1SIDL = 0x42,
  MCP_TXB1EID8 = 0x43,
  MCP_TXB1EID0 = 0x44,
  MCP_TXB1DLC  = 0x45,
  MCP_TXB1DATA = 0x46,
  MCP_TXB2CTRL = 0x50,
  MCP_TXB2SIDH = 0x51,
  MCP_TXB2SIDL = 0x52,
  MCP_TXB2EID8 = 0x53,
  MCP_TXB2EID0 = 0x54,
  MCP_TXB2DLC  = 0x55,
  MCP_TXB2DATA = 0x56,
  MCP_RXB0CTRL = 0x60,
  MCP_RXB0SIDH = 0x61,
  MCP_RXB0SIDL = 0x62,
  MCP_RXB0EID8 = 0x63,
  MCP_RXB0EID0 = 0x64,
  MCP_RXB0DLC  = 0x65,
  MCP_RXB0DATA = 0x66,
  MCP_RXB1CTRL = 0x70,
  MCP_RXB1SIDH = 0x71,
  MCP_RXB1SIDL = 0x72,
  MCP_RXB1EID8 = 0x73,
  MCP_RXB1EID0 = 0x74,
  MCP_RXB1DLC  = 0x75,
  MCP_RXB1DATA = 0x76
} McpRegister;


/* -------- MCP2515 private function declarations ----------- */

static PassThruError init(void *_this, void *_params);
static PassThruError connect(void *_this, PassThruParams *params);
static PassThruError disconnect(void *_this);
static PassThruError sendMsg(void *_this, PassThruParams *params);
static PassThruError receiveMsg(void *_this, PassThruParams *params);
static PassThruError setFilter(void *_this, PassThruParams *params);
static PassThruError resetFilter(void *_this, PassThruParams *params);
static PassThruError handleIoctl(void *_this, PassThruParams *params);

static uint8_t isConnected(void *this);
static uint8_t isCapableOf(void *this, PassThruProtocolId protocol);

static void startSPI(CAN_MCP2515_TypeDef *this);
static void stopSPI(CAN_MCP2515_TypeDef *this);
static void reset(CAN_MCP2515_TypeDef *this);
static void readRegister(CAN_MCP2515_TypeDef *this, McpRegister reg, uint8_t *buf);
static void writeRegister(CAN_MCP2515_TypeDef *this, McpRegister reg, uint8_t val);
static void writeRegisters(CAN_MCP2515_TypeDef *this, McpRegister reg, uint8_t *vals, uint8_t count);
static void bitSetRegister(CAN_MCP2515_TypeDef *this, McpRegister reg, uint8_t mask, uint8_t val);


/* ---------- MCP2515 public function definitions ----------- */

void MCP2515_getInterface(PassThruPeriph_ItfTypeDef *itf)
{
  PassThruPeriph_ItfTypeDef _itf = {
    init,
    connect,
    disconnect,
    receiveMsg,
    sendMsg,
    setFilter,
    resetFilter,
    handleIoctl,
    isConnected,
    isCapableOf
  };
  *itf = _itf;
}

/* ---------- MCP2515 private function definitions ----------- */

static void startSPI(CAN_MCP2515_TypeDef *this)
{
  HAL_GPIO_WritePin(this->GPIO, this->GPIO_CS_Pin, GPIO_PIN_RESET);
}

static void stopSPI(CAN_MCP2515_TypeDef *this)
{
  HAL_GPIO_WritePin(this->GPIO, this->GPIO_CS_Pin, GPIO_PIN_RESET);
}

static void reset(CAN_MCP2515_TypeDef *this)
{
  McpCommand cmd = MCP_RESET;
  startSPI(this);
  HAL_SPI_Transmit(this->hspi, &cmd, 1, 1);
  stopSPI(this);
}

static void readRegister(CAN_MCP2515_TypeDef *this, McpRegister reg, uint8_t *buf)
{
  startSPI(this);
  uint8_t sendBuf[] = { MCP_READ, reg };
  HAL_SPI_Transmit(this->hspi, sendBuf, sizeof(sendBuf), 1);
  HAL_SPI_Receive(this->hspi, buf, 1, 1);
  stopSPI(this);
}

static void writeRegister(CAN_MCP2515_TypeDef *this, McpRegister reg, uint8_t val)
{
  startSPI(this);
  uint8_t sendBuf[] = { MCP_WRITE, reg, val };
  HAL_SPI_Transmit(this->hspi, sendBuf, sizeof(sendBuf), 1);
  stopSPI(this);
}

static void writeRegisters(CAN_MCP2515_TypeDef *this, McpRegister reg, uint8_t *vals, uint8_t count)
{
  startSPI(this);
  uint8_t sendBuf[] = { MCP_WRITE, reg };
  HAL_SPI_Transmit(this->hspi, sendBuf, sizeof(sendBuf), 1);
  HAL_SPI_Transmit(this->hspi, vals, count, 2);
  stopSPI(this);
}

static void bitSetRegister(CAN_MCP2515_TypeDef *this, McpRegister reg, uint8_t mask, uint8_t val)
{
  startSPI(this);
  uint8_t sendBuf[] = { MCP_BIT_MODIFY, reg, mask, val };
  HAL_SPI_Transmit(this->hspi, sendBuf, sizeof(sendBuf), 1);
  stopSPI(this);
}


static PassThruError init(void *_this, void *_params)
{
  CAN_MCP2515_TypeDef *this = _this;
  CAN_MCP2515_InitTypeDef *init = _params;

  this->hspi = init->hspi;
  this->GPIO = init->GPIO;
  this->GPIO_CS_Pin = init->GPIO_Pin;

  this->connected = 0;

  // Controller reset
  reset(this);
  HAL_Delay(2);

  // Check if there is MCP2515 connected
  uint8_t readBuf[1];
  readRegister(this, MCP_CANCTRL, readBuf);

  if (readBuf[0] != 0x87) {
    Error_Handler();
    return ERR_FAILED;
  }

  bitSetRegister(this, MCP_RXB0CTRL, RXBNCTRL_RXM_MASK, 0b11 << RXBNCTRL_RXM);

  uint8_t result[1];
  readRegister(this, MCP_RXB0CTRL, result);
  readRegister(this, MCP_CANCTRL, result);


}

static PassThruError connect(void *_this, PassThruParams *params)
{

}

static PassThruError disconnect(void *_this)
{

}

static PassThruError sendMsg(void *_this, PassThruParams *params)
{

}

static PassThruError receiveMsg(void *_this, PassThruParams *params)
{

}

static PassThruError setFilter(void *_this, PassThruParams *params)
{

}

static PassThruError resetFilter(void *_this, PassThruParams *params)
{

}

static PassThruError handleIoctl(void *_this, PassThruParams *params)
{

}

static uint8_t isConnected(void *this)
{

}

static uint8_t isCapableOf(void *this, PassThruProtocolId protocol)
{

}
