#include "XBee.h"
#include "stm32l4xx_hal.h"
#include "string.h"

UART_HandleTypeDef* huart_g;

void put_int16(uint8_t* pos, uint16_t num) {
	pos[0] = (num & 0xFF00) >> 8;
	pos[1] = num & 0xFF;
}


void init_XBee(UART_HandleTypeDef* huart) {
	huart_g = huart;
}

void send_XBee(uint8_t *data, size_t len) {
	uint8_t buf[XBEE_BUFFER_LENGTH];
	uint16_t sum = 0;

	buf[0] = 0x7E;
	put_int16(buf + 1, len);
	for (size_t i = 0; i < len; i++) {
		buf[i + 3] = data[i];
		sum += data[i];
		sum &= 0xFF;
	}
	buf[len + 3] = 0xFF - sum;
	HAL_UART_Transmit(huart_g, buf, len + 4, 100);
}

void concat_XBee(uint8_t *buf, char* data, size_t offset) {
	for (size_t i = 0; data[i]; i++) {
		buf[i + offset] = data[i];
	}
}

void tx_req_XBee(uint8_t id, uint16_t addr, uint8_t opt, char* data) {
	uint8_t buf[XBEE_BUFFER_LENGTH];

	buf[0] = 0x01;
	buf[1] = id;
	put_int16(buf + 2, addr);
	buf[4] = opt;

	concat_XBee(buf, data, 5);
	send_XBee(buf, strlen(data) + 5);
}

void tx_at(char* cmd) {
	uint8_t buf[XBEE_BUFFER_LENGTH];

//	buf[0] = 0x08;
//	buf[1] = id;
//	put_int16(buf + 2, addr);
//	buf[4] = opt;
//
//	concat_XBee(buf, data, 5);
//	send_XBee(buf, strlen(data) + 5);
}
