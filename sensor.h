

#ifndef __SENSOR_H
#define	__SENSOR_H

#include "nrf.h"

void sensor_module_init();

void sensor_msg_handle(uint8_t msg_id,uint8_t *msg_buf);

#endif /* __SENSOR_H */






