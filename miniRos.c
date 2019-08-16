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
#include "message.h"
#include "sensor.h"
static xQueueHandle miniros_cmd_queue;
static TaskHandle_t miniros_process_task_t;

static MINIROS_DEV_LIST_S *p_dev_list = NULL;
MINIROS_DEV_LIST_S *miniros_cmd_list_create(MINIROS_DEVICE_S *device)
{
    MINIROS_DEV_LIST_S *list = (MINIROS_DEV_LIST_S *)malloc(sizeof(MINIROS_DEV_LIST_S));
    memcpy(&list->device,device,sizeof(MINIROS_DEVICE_S));
    list->next = NULL;
    return list;

}

void miniros_dev_register(uint8_t *dev_name,void (*handler)(uint8_t cmd_id,uint8_t *cmd_buf))
{
//      assert(p_msg_module_list != NULL);
    MINIROS_DEVICE_S dev;
    memcpy(dev.name,dev_name,strlen(dev_name)+1);
    dev.handler = handler;
    MINIROS_DEV_LIST_S *node = miniros_cmd_list_create(&dev);
    if (p_dev_list == NULL){
        p_dev_list = node;
        return;
    }
    
    MINIROS_DEV_LIST_S* cur = p_dev_list;
    while (cur->next != NULL){
        cur = cur->next;
    }
    cur->next = node;
}

void miniros_dev_unregister(uint8_t *dev_name)
{
    MINIROS_DEV_LIST_S *cur = p_dev_list;
    MINIROS_DEV_LIST_S *prev = NULL;

    if (p_dev_list == NULL)
        return;
    while (cur){
        if(0 == strcmp(cur->device.name,dev_name)){
            if (p_dev_list == cur){
                p_dev_list = cur->next;
                free(cur);
                cur = NULL;
            } else {
                prev->next = cur->next;
                free(cur);
                cur = NULL;
            }
            break;
        }
        prev = cur;
        cur = cur->next;
    }
}

void miniros_dev_destroy()
{
    MINIROS_DEV_LIST_S *cur = NULL;
    MINIROS_DEV_LIST_S *del = NULL;
//    assert(p_dev_list);
    cur = p_dev_list;
    while (cur){
        del = cur;
        cur = cur->next;
        free(del);
        del = NULL;
    }
    p_dev_list = NULL;
}

MINIROS_DEV_LIST_S *miniros_dev_find(MINIROS_DEV_LIST_S *list_first, uint8_t *dev_name)
{
    for (MINIROS_DEV_LIST_S *cur = list_first; cur != NULL; cur = cur->next){
        LOG( "Queue dev module:%s, %s!\n",cur->device.name,dev_name );
        if(0 == strcmp(cur->device.name,dev_name)){
            return cur;
        }
    }

    return NULL;
}

void miniros_cmd_send(uint8_t *dev, uint8_t cmd_id, uint8_t *cmd_buf)
{
    BaseType_t xStatus;
    MINIROS_CMD_S cmd;
    memcpy(cmd.dev,dev,strlen(dev)+1);
    cmd.cmd_id = cmd_id;
    memcpy(cmd.cmd_buf,cmd_buf,strlen(cmd_buf)+1);
    LOG( "miniros_cmd_send:%s,%d,%s!\n",dev,cmd_id,cmd_buf );
    xStatus = xQueueSendFromISR( miniros_cmd_queue, &cmd, 0);
    if( xStatus != pdPASS )
         LOG("miniros_cmd_send fail: %d\n",xStatus);

}

static void miniros_process_task(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    MINIROS_CMD_S miniros_cmd;
    BaseType_t xStatus;
    uint8_t cmd_cnt = 0;
    MINIROS_DEV_LIST_S *find_dev_list;
    while(1)
    {
        cmd_cnt = uxQueueMessagesWaiting( miniros_cmd_queue );
        LOG( "Queue cmd cnt:%d!\n",cmd_cnt );

        xStatus = xQueueReceive( miniros_cmd_queue, &miniros_cmd, portMAX_DELAY);
        LOG( "Queue cmd xStatus:%d!\n",xStatus );
        if( xStatus == pdPASS ){
            LOG( "Queue cmd device:%s,%d,%s!\n",miniros_cmd.dev,miniros_cmd.cmd_id,miniros_cmd.cmd_buf );
            find_dev_list = miniros_dev_find(p_dev_list,miniros_cmd.dev);
            if(find_dev_list)
                find_dev_list->device.handler(miniros_cmd.cmd_id,miniros_cmd.cmd_buf);
            else
                LOG( "Queue msg list cannot find module!\n" );
        }
        vTaskDelay(1000);
    }
}

void miniros_init()
{

    miniros_msg_init();
    sensor_module_init();

    //miniRos drivers init.
    nrf52840_init();
    oled_init();
    lobot_init();
    wifi_init();
    us026_init();



    miniros_cmd_queue = xQueueCreate( MINIROS_CMD_QUEUE_MAX , sizeof( MINIROS_CMD_S) );

    /* Create task for robot module process with priority set to 2 */
    xTaskCreate(miniros_process_task,"miniros process",MINIROS_PROCESS_TASK_STACK_SIZE,NULL,MINIROS_PROCESS_TASK_PRIO,&miniros_process_task_t);
    LOG("miniros_init!\n");
}

/** @} */
