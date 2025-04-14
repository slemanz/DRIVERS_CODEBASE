#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

void serial_init(void);

bool serial_available(void);
bool serial_buffer_write(uint8_t data);

void serial_send_byte(uint8_t *data, uint32_t Len);
uint8_t serial_rcv_byte(void);

void serial_send(uint8_t *data, uint32_t Len);
bool serial_rcv(uint8_t *data, uint32_t Len);


#endif /* INC_SERIAL_H_ */