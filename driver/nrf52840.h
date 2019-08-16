#ifndef __NRF52840_H
#define	__NRF52840_H
#include <stdint.h>
#include <string.h>

void nrf52840_init();
void bt_send_data( uint8_t   * p_data,
                   uint16_t  length);
#endif /* __NRF52840_H */






