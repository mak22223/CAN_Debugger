/*
 * VCPCommParser.h
 *
 *  Created on: 23 дек. 2022 г.
 *      Author: mak22
 */

#ifndef COMCOMMANDPARSER_H_
#define COMCOMMANDPARSER_H_


/* ------------------ VCP command parser includes ----------------- */

#include "usbd_cdc_if.h"
#include <PassThru/PassThru_if.h>

/*
typedef struct {
  uint8_t (* Init)(void);
  uint8_t (* DeInit)(void);
  uint8_t (* ReceiveCmd)(PassThruCommand *cmd, PassThruParams *params);
  uint8_t (* SendAnswer)(PassThruCommand *cmd, PassThruAnswer *ans);
} PassThruComm_ItfTypeDef;
*/


/* ---------- VCP command parser public function declaration ----------- */

uint8_t init(void);
uint8_t deinit(void);
uint8_t receiveCmd(uint8_t *cmd, PassThruParams *params);
uint8_t sendAnswer(uint8_t *cmd, PassThruAnswer *ans);

/* ---------- VCP command parser private function declaration ----------- */



#endif /* COMCOMMANDPARSER_H_ */
