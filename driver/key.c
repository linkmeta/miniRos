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
		  /*����Ƿ��а������� */
   	if( nrf_gpio_pin_read(KEY_1)== 0 ) 
	  {	   
	 	   /*��ʱ����*/
	  	 Delay(10000);		
	   		if(nrf_gpio_pin_read(KEY_1)== 0 )  
					{	 
						/*�ȴ������ͷ� */
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
		  /*����Ƿ��а������� */
   	if( nrf_gpio_pin_read(KEY_2)== 0 ) 
	  {	   
	 	   /*��ʱ����*/
	  	 Delay(10000);		
	   		if(nrf_gpio_pin_read(KEY_2)== 0 )  
					{	 
						/*�ȴ������ͷ� */
						while(nrf_gpio_pin_read(KEY_2)== 0 );   
			 			return 	0	; 
					}
			else
					return 1;
		}
	else
		return 1;
}
