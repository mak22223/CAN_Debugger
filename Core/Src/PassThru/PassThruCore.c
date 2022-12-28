/*
 * Control.c
 *
 *  Created on: 23 дек. 2022 г.
 *      Author: mak22
 */

#include <PassThru/PassThruCore.h>
#include "PassThru/PassThruPeriph_if.h"
#include "usb_vcp/VCPCommParser.h"
#include "MCP2515/MCP2515.h"

/* -------------- PassThruСore private defines ------------------ */

#define PERIPH_COUNT 2

/* -------- PassThruСore private functions declarations ----------- */

static PassThruError ConnectHandler(PassThruParams *params);
static PassThruError DisconnectHandler(PassThruParams *params);
static PassThruError ReadMsgsHandler(PassThruParams *params);
static PassThruError WriteMsgsHandler(PassThruParams *params);
static PassThruError StartPeriodicMsgsHandler(PassThruParams *params);
static PassThruError StopPeriodicMsgsHandler(PassThruParams *params);
static PassThruError StartMsgFilterHandler(PassThruParams *params);
static PassThruError StopMsgFilterHandler(PassThruParams *params);
static PassThruError SetProgrammingVoltageHandler(PassThruParams *params);
static PassThruError GetLastErrorHandler(PassThruParams *params);
static PassThruError IoctlHandler(PassThruParams *params);


/* ------------- PassThruСore private variables ------------------- */

PassThruComm_ItfTypeDef comm_itf;

struct {
  void *periph;
  PassThruPeriph_ItfTypeDef itf;
} periphs[PERIPH_COUNT];

CAN_MCP2515_TypeDef Can1;
CAN_MCP2515_TypeDef Can2;

PassThruCommand command_id;
PassThruParams param_buf;
uint8_t *last_error_string;

const uint8_t error_strings[1][1] = {
    ""
};

/* ------------- PassThru functions definition ------------------- */

/*
 * Init PassThruCore, set communication interface
 * @retval None
 */
void PassThru_init(SPI_HandleTypeDef* _hspi)
{
  VCP_getInterface(&comm_itf);

  periphs[0].periph = &Can1;
  periphs[1].periph = &Can2;
  MCP2515_getInterface(&periphs[0].itf);
  MCP2515_getInterface(&periphs[1].itf);

  CAN_MCP2515_InitTypeDef can1_init = {
    _hspi,
    CAN1_CS_GPIO_Port,
    CAN1_CS_Pin
  };

  CAN_MCP2515_InitTypeDef can2_init = {
      _hspi,
      CAN2_CS_GPIO_Port,
      CAN2_CS_Pin
  };

  periphs[0].itf.Init(&Can1, &can1_init);
  periphs[1].itf.Init(&Can2, &can2_init);

}

/*
 * Serve PassThru events
 * @retval None
 */
void PassThru_tick(void)
{
  if(comm_itf.ReceiveCmd(&command_id, &param_buf) == IF_OK) {
    switch (command_id) {
      case NO_COMMAND:

        break;

      case CONNECT:
//        ConnectHandler(&param_buf);
        break;

      case DISCONNECT:

        break;

      case READ_MSGS:

        break;

      case WRITE_MSGS:

        break;

      case START_PERIODIC_MSG:

        break;

      case STOP_PERIODIC_MSG:

        break;

      case START_MSG_FILTER:

        break;

      case STOP_MSG_FILTER:

        break;

      case SET_PROGRAMMING_VOLTAGE:

        break;

      case READ_VERSION:

        break;

      case GET_LAST_ERROR:

        break;

      case IOCTL:

        break;

      default:
        Error_Handler();
        break;
    }

  }
}
