#include "key.h"


void KEY_Init(void)
{
	
  nrf_gpio_cfg_input(13,NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(14,NRF_GPIO_PIN_PULLUP);
	}
	
 void Delay(uint32_t temp)
{
  for(; temp!= 0; temp--);
} 


uint8_t KEY1_Down(void)
{			
		  /*检测是否有按键按下 */
   	if( nrf_gpio_pin_read(KEY_1)== 0 ) 
	  {	   
	 	   /*延时消抖*/
	  	 Delay(10000);		
	   		if(nrf_gpio_pin_read(KEY_1)== 0 )  
					{	 
						/*等待按键释放 */
						while(nrf_gpio_pin_read(KEY_1)== 0 );   
			 			return 	0	; 
					}
			else
					return 1;
		}
	else
		return 1;
}


uint8_t KEY2_Down(void)
{			
		  /*检测是否有按键按下 */
   	if( nrf_gpio_pin_read(KEY_2)== 0 ) 
	  {	   
	 	   /*延时消抖*/
	  	 Delay(10000);		
	   		if(nrf_gpio_pin_read(KEY_2)== 0 )  
					{	 
						/*等待按键释放 */
						while(nrf_gpio_pin_read(KEY_2)== 0 );   
			 			return 	0	; 
					}
			else
					return 1;
		}
	else
		return 1;
}
