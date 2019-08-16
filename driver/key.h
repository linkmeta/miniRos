#ifndef __KEY_H
#define	__KEY_H

#include "nrf52.h"
#include "nrf_gpio.h"
#define KEY_1  13
#define KEY_2  14
 
void KEY_Init(void);
uint8_t KEY1_Down(void);
uint8_t KEY2_Down(void);
#endif /* __KEY_H */

