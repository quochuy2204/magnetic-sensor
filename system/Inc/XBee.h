#ifndef XBEE_H
#define XBEE_H

#include "stm32l4xx_hal.h"

#define XBEE_BUFFER_LENGTH 100

void init_XBee(UART_HandleTypeDef* huart);
void tx_req_XBee(uint8_t id, uint16_t addr, uint8_t opt, char* data);

#endif
