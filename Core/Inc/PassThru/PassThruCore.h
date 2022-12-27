/*
 * Control.h
 *
 *  Created on: 23 дек. 2022 г.
 *      Author: mak22
 */

#ifndef INC_PASSTHRU_PASSTHRUCORE_H_
#define INC_PASSTHRU_PASSTHRUCORE_H_

/* ------------------ PassThru includes ------------------- */

#include "PassThru/PassThru_def.h"
#include "PassThru/PassThruComm_if.h"


/* ------------- PassThru structs declaration ------------------- */



/* ------------- PassThru functions declaration ------------------- */

/*
 * Init PassThruCore, set communication interface
 * @retval None
 */
void PassThru_init(PassThruComm_ItfTypeDef *itf);
void PassThru_tick();

#endif /* INC_PASSTHRU_PASSTHRUCORE_H_ */
