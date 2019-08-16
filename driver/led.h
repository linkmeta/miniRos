#ifndef __LED_H
#define	__LED_H

#include "nrf52840.h"


#define LED_0          13
#define LED_1          14
#define LED_2          15
#define LED_3          16


void LED_Init(void);
void LED1_Open(void);
void LED1_Close(void);
void LED1_Toggle(void);
void LED2_Open(void);
void LED2_Close(void);
void LED2_Toggle(void);
void LED3_Open(void);
void LED3_Close(void);
void LED3_Toggle(void);


#endif /* __LED_H */

