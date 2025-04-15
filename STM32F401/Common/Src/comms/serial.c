#include <stddef.h>
#include "comms/serial.h"
#include "utils/ring-buffer.h"


static serial_send_byte_fn _serial_send_byte = NULL;

static ring_buffer_t rb = {0U};
static uint8_t data_buffer[SERIAL_BUFFER_SIZE] = {0U};

void serial_init(serial_send_byte_fn send)
{
    _serial_send_byte = send;
    ring_buffer_setup(&rb, data_buffer, SERIAL_BUFFER_SIZE);
}


bool serial_available(void)
{
    return !ring_buffer_empty(&rb);
}

bool serial_buffer_write(uint8_t data)
{
    return ring_buffer_write(&rb, data);
}

uint8_t serial_rcv_byte(void)
{
    uint8_t byte = 0;
    (void)serial_rcv(&byte, 1);
	return byte;
}

uint32_t serial_rcv(uint8_t *data, uint32_t Len)
{
    if(Len == 0)
	{
		return 0;
	}
	
	for(uint32_t bytes_read = 0; bytes_read < Len; bytes_read++)
	{
		if(!ring_buffer_read(&rb, &data[bytes_read]))
		{
			return bytes_read;
		}
	}

	return Len;
}

void serial_send_byte(uint8_t data)
{
    _serial_send_byte(data);
}

void serial_send(uint8_t *data, uint32_t Len)
{
    if(Len == 0) return;
    uint32_t LenSend = 0;
    do
    {
        _serial_send_byte(data[LenSend]);
        LenSend++;
        Len--;
    } while (Len);
}