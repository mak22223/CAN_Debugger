/*
 * PassThru_if.h
 *
 *  Created on: 23 дек. 2022 г.
 *      Author: mak22
 */

#ifndef INC_PASSTHRU_PASSTHRU_IF_H_
#define INC_PASSTHRU_PASSTHRU_IF_H_

/* ---------------- PassThruInterface includes ------------------- */

#include "stdint.h"


/* ------------- PassThruInterface enums declaration ------------------- */

typedef enum {
  CONNECT = 0U,
  DISCONNECT,
  READ_MSGS,
  WRITE_MSGS,
  START_PERIODIC_MSG,
  STOP_PERIODIC_MSG,
  START_MSG_FILTER,
  STOP_MSG_FILTER,
  SET_PROGRAMMING_VOLTAGE,
  READ_VERSION,
  GET_LAST_ERROR,
  IOCTL
} PassThruCommand;

typedef enum {
  STATUS_NOERROR = 0U,
  ERR_NOT_SUPPORTED,
  ERR_INVALID_CHANNEL_ID,
  ERR_INVALID_PROTOCOL_ID,
  ERR_NULLPARAMETER,
  ERR_INVALID_IOCTL_VALUE,
  ERR_INVALID_FLAGS,
  ERR_FAILED,
  ERR_DEVICE_NOT_CONNECTED,
  ERR_TIMEOUT,
  ERR_INVALID_MSG,
  ERR_INVALID_TIME_INTERVAL,
  ERR_EXCEEDED_LIMIT,
  ERR_INVALID_MSG_ID,
  ERR_INVALID_ERROR_ID,
  ERR_INVAILD_IOCTL_ID,
  ERR_BUFFER_EMPTY,
  ERR_BUFFER_FULL,
  ERR_BUFFER_OVERFLOW,
  ERR_PIN_INVALID,
  ERR_CHANNEL_IN_USE,
  ERR_MSG_PROTOCOL_ID
} PassThruError;

typedef enum {
  IF_OK = 0U,
  IF_ERROR
} PassThruIfError;

/* ------------- PassThruInterface structs declaration ------------------- */

typedef union {
  struct {

  } Connect;
  struct  {

  } Disconnect;
  struct  {

  } ReadMsgs;
  struct {

  } WriteMsgs;
  struct {

  } StartPeriodicMsg;
  struct {

  } StopPeriodicMsg;
  struct {

  } StartMsgFilter;
  struct {

  } StopMsgFilter;
  struct {

  } SetProgrammingVoltage;
  struct {

  } ReadVersion;
  struct {

  } GetLastVersion;
  struct {

  } IOCTL;
} PassThruParams;

typedef struct {
  uint32_t errorCode;

  union {
    struct {

    } Connect;
    struct {

    } Disconnect;
    struct {

    } ReadMsgs;
    struct {

    } WriteMsgs;
    struct {

    } StartPeriodicMsg;
    struct {

    } StopPeriodicMsg;
    struct {

    } StartMsgFilter;
    struct {

    } StopMsgFilter;
    struct {

    } SetProgrammingVoltage;
    struct {

    } ReadVersion;
    struct {

    } GetLastVersion;
    struct {

    } IOCTL;
  };

} PassThruAnswer;

typedef struct {
  uint32_t ProtocolID;
  uint32_t RxStatus;
  uint32_t TxFlags;
  uint32_t Timestamp;
  uint32_t DataSize;
  uint32_t ExtraDataIndex;
  uint8_t Data[4128];
} PassThruMessage;

typedef struct {
  uint8_t (* Init)(void);
  uint8_t (* DeInit)(void);
  uint8_t (* ReceiveCmd)(uint8_t *cmd, PassThruParams *params);
  uint8_t (* SendAnswer)(uint8_t *cmd, PassThruAnswer *ans);
} PassThruComm_ItfTypeDef;

#endif /* INC_PASSTHRU_PASSTHRU_IF_H_ */
