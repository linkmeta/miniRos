/*******************************************************************************
* Copyright 2018~2019 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
#  @file miniRos.c
#  @author qqnokia@gmail.com
#******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "utils.h"
#include "miniRos.h"
#include "nrf52840.h"
#include "oled.h"
#include "utils.h"
#include "wifi.h"
#include "us_026.h"
#include "lobot.h"

xQueueHandle miniros_cmd_queue;
static TaskHandle_t miniros_process_task_t;

static void miniros_process_task(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    MINIROS_CMD_S miniros_cmd;
    BaseType_t xStatus;
    uint8_t cmd_cnt = 0;
    while(1)
    {
        cmd_cnt = uxQueueMessagesWaiting( miniros_cmd_queue );
        LOG( "Queue cnt:%d!\n",cmd_cnt );

        xStatus = xQueueReceive( miniros_cmd_queue, &miniros_cmd, portMAX_DELAY);

        if( xStatus == pdPASS ){
            LOG( "Queue cmd id:%d!\n",miniros_cmd.cmd_id );
            switch(miniros_cmd.cmd_id){

                case WIFI:/*WIFI Module control*/
                wifi_msg_process(miniros_cmd.cmd_buf);
                break;

                case OLED:/*OLED Module control*/
                oled_msg_process(miniros_cmd.cmd_buf);
                break;

                case US:
//                us026_get_distance();

                case LOBOT:
                lobot_msg_process(miniros_cmd.cmd_buf);
  
                case BT:
                lobot_msg_process(miniros_cmd.cmd_buf);
                default:
                break;

            }
        }
        vTaskDelay(1000);
    }
}

void miniros_init()
{
    //miniRos drivers init.
    nrf52840_init();
    oled_init();
    lobot_init();
    wifi_init();
    us026_init();

    miniros_msg_init();
    sensor_module_init();

    miniros_cmd_queue = xQueueCreate( MINIROS_CMD_QUEUE_MAX , sizeof( MINIROS_CMD_S) );

    /* Create task for robot module process with priority set to 2 */
    xTaskCreate(miniros_process_task,"miniros process",MINIROS_PROCESS_TASK_STACK_SIZE,NULL,MINIROS_PROCESS_TASK_PRIO,&miniros_process_task_t);
    LOG("miniros_init!\n");
}

/** @} */
