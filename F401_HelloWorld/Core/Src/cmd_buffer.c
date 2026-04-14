/*
 * cmd_buffer.c
 *
 *  Created on: Apr 6, 2026
 *      Author: Likhita
 */


#include "cmd_buffer.h"

BleCommandQueue_t  cmdBuffer = {0};

uint8_t cbfifo_ble_write(BleCommandMsg *ble_msg)
{
    if(cmdBuffer.count >= BUFFER_SIZE)
    {
        cmdBuffer.tail = (cmdBuffer.tail + 1) % BUFFER_SIZE;
        cmdBuffer.count--;
    }
    cmdBuffer.buffer[cmdBuffer.head] = *ble_msg;
    cmdBuffer.head = (cmdBuffer.head + 1) % BUFFER_SIZE ;
    cmdBuffer.count++;
    return 1;
}

uint8_t cbfifo_ble_read(BleCommandMsg *ble_msg)
{
    if(cmdBuffer.count == 0)
    {
        return 0;
    }
    *ble_msg = cmdBuffer.buffer[cmdBuffer.tail];
    cmdBuffer.tail = (cmdBuffer.tail + 1) % BUFFER_SIZE;
    cmdBuffer.count--;
    return 1;
}
