

#ifndef __SENSOR_H
#define	__SENSOR_H

#include "nrf.h"

typedef enum  sensor_msg_e{
    DISTANCE_INFO = 0,


}SENSOR_MSG_E;

void sensor_module_init();

void sensor_msg_handle(uint8_t msg_id,uint8_t *msg_buf);

#endif /* __SENSOR_H */






