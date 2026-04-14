/*
 * cmd_buffer.h
 *
 *  Created on: Apr 6, 2026
 *      Author: Likhita
 */

#ifndef INC_CMD_BUFFER_H_
#define INC_CMD_BUFFER_H_

#include <stdint.h>

#define BUFFER_SIZE 256

typedef enum {
    CMD_NONE = 0,
    CMD_HOVER,
    CMD_MOVE_LEFT,
    CMD_MOVE_RIGHT,
    CMD_LAND,
    CMD_TAKEOFF
} BleCommand_t;

typedef struct {
    BleCommand_t cmd;
    float param1;
} BleCommandMsg;

typedef struct{
	BleCommandMsg buffer[BUFFER_SIZE];
    volatile uint8_t head; //writer index
    volatile uint8_t tail; //reader index
    volatile uint8_t count;
}BleCommandQueue_t;


extern BleCommandQueue_t cmdBuffer;

uint8_t cbfifo_ble_write(BleCommandMsg *msg);
uint8_t cbfifo_ble_read(BleCommandMsg *msg);

#endif /* INC_CMD_BUFFER_H_ */
