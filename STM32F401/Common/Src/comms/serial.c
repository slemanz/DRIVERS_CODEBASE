#include <stddef.h>
#include "comms/serial.h"
#include "utils/ring-buffer.h"

static serial_send_byte_fn _serial_send_byte = NULL;

void serial_init(serial_send_byte_fn send)
{
    _serial_send_byte = send;
}

void serial_send_byte(uint8_t data)
{
    _serial_send_byte(data);
}


