#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

#define SERIAL_BUFFER_SIZE 128

typedef void (*serial_send_byte_fn)(uint8_t byte);

void serial_init(serial_send_byte_fn send);

bool serial_available(void);
bool serial_buffer_write(uint8_t data);

void serial_send_byte(uint8_t data);
uint8_t serial_rcv_byte(void);

void serial_send(uint8_t *data, uint32_t Len);
uint32_t serial_rcv(uint8_t *data, uint32_t Len);


#endif /* INC_SERIAL_H_ */