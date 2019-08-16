/*******************************************************************************
* Copyright 2018~2019 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
#  @file sensor.c
#  @author qqnokia@gmail.com
#******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "utils.h"
#include "message.h"
#include "sensor.h"
void sensor_msg_handle(uint8_t msg_id,uint8_t *msg_buf)
{
    ret_code_t ret;

//    ret = nrf_drv_clock_init();
//    APP_ERROR_CHECK(ret);
//    ret = nrf_drv_power_init(NULL);
//    APP_ERROR_CHECK(ret);
//
//    nrf_drv_clock_lfclk_request(NULL);
//    ret = app_timer_init();
//    APP_ERROR_CHECK(ret);
//
//    // Initialize LEDs and buttons.
//    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);
    LOG("sensor_msg_handle:%d,%d!\n",msg_id,atoi(msg_buf));
//    ret = nrf_serial_init(&serial0_uarte, &m_uarte0_drv_config, &serial0_config);
//    APP_ERROR_CHECK(ret);


//    char tx_message[] = "AT\r\n";
//    wifi_cmd_send(msg_buf);
//    ret = nrf_serial_write(&serial1_uarte,
//                           tx_message,
//                           strlen(tx_message),
//                           NULL,
//                           NRF_SERIAL_MAX_TIMEOUT);
//    (void)nrf_serial_flush(&serial1_uarte, 0);

//    wifi_recv_task(NULL); 
}

void sensor_module_init()
{
//    MINIROS_MODULE_S module;
//    char module_name[] = "sensor";
//    xTaskCreate(miniros_msg_task,"miniros msg task",MINIROS_MSG_TASK_STACK_SIZE,NULL,MINIROS_MSG_TASK_PRIO,&miniros_msg_task_t);
    
//    memcpy(module.name,module_name,strlen(module_name)+1);
//    module.handler = sensor_msg_handle;
    miniros_msg_module_register("sensor",sensor_msg_handle);
    LOG("sensor module init!\n");
}
