

#ifndef __OLED_H
#define	__OLED_H

#include "nrf.h"

#define LCD_SCL  NRF_GPIO_PIN_MAP(1,15)
#define LCD_SDA  NRF_GPIO_PIN_MAP(1,11)
#define LCD_RST  NRF_GPIO_PIN_MAP(1,12)
#define LCD_DC   NRF_GPIO_PIN_MAP(1,14)
#define LCD_CS   NRF_GPIO_PIN_MAP(1,10)
#define LCD_POWER_EN   NRF_GPIO_PIN_MAP(1,13)

#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel	    ((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		  64
#define	Brightness	0xCF 
#define X_WIDTH 128
#define Y_WIDTH 64

void oled_init();
void oled_show_info(uint8_t id,uint8_t *buf);

void oled_msg_process(uint8_t *msg_buf);

#endif /* __OLED_H */






