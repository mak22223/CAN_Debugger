/*
 * MCP2515.h
 *
 *  Created on: Dec 28, 2022
 *      Author: mak22
 */

#ifndef INC_MCP2515_MCP2515_H_
#define INC_MCP2515_MCP2515_H_

/* ------------------ MCP2515 includes ------------------- */

#include "PassThru/PassThruPeriph_if.h"
#include "stm32f4xx_hal.h"




/* ----------- MCP2515 public struct declaration ------------ */

typedef struct {
  SPI_HandleTypeDef* hspi;
  GPIO_TypeDef* GPIO;
  uint16_t GPIO_CS_Pin;

  uint8_t connected;
} CAN_MCP2515_TypeDef;

typedef struct {
  SPI_HandleTypeDef* hspi;
  GPIO_TypeDef* GPIO;
  uint16_t GPIO_Pin;

} CAN_MCP2515_InitTypeDef;



/* ----------- MCP2515 public function declaration ------------ */

void MCP2515_getInterface(PassThruPeriph_ItfTypeDef *itf);

#endif /* INC_MCP2515_MCP2515_H_ */
