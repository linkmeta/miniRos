/*******************************************************************************
* Copyright 2018~2019 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
#  @file  message.c
#  @author qqnokia@gmail.com
#******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "utils.h"
#include "message.h"
//

static xQueueHandle miniros_msg_queue;
static TaskHandle_t miniros_msg_task_t;
//StreamBufferHandle_t wifi_recv_buf_t;
static MINIROS_MSG_MODULE_LIST_S *p_msg_module_list = NULL;
MINIROS_MSG_MODULE_LIST_S *miniros_msg_module_list_create(MINIROS_MODULE_S *module)
{
    MINIROS_MSG_MODULE_LIST_S *list = (MINIROS_MSG_MODULE_LIST_S *)malloc(sizeof(MINIROS_MSG_MODULE_LIST_S));
    memcpy(&list->module,module,sizeof(MINIROS_MODULE_S));
    list->next = NULL;
    return list;

}

void miniros_msg_module_register(uint8_t *module_name,void (*handler)(uint8_t msg_id,uint8_t *msg_buf))
{
//      assert(p_msg_module_list != NULL);
    MINIROS_MODULE_S module;
    memcpy(module.name,module_name,strlen(module_name)+1);
    module.handler = handler;
    MINIROS_MSG_MODULE_LIST_S *node = miniros_msg_module_list_create(&module);
    if (p_msg_module_list == NULL)
    {
        p_msg_module_list = node;
        return;
    }
    
    MINIROS_MSG_MODULE_LIST_S* cur = p_msg_module_list;
    while (cur->next != NULL)
    {
        cur = cur->next;
    }
    cur->next = node;
}

void miniros_msg_module_unregister(uint8_t *module_name)
{
    MINIROS_MSG_MODULE_LIST_S *cur = p_msg_module_list;
    MINIROS_MSG_MODULE_LIST_S *prev = NULL;

    if (p_msg_module_list == NULL)
        return;
    while (cur)
    {
        if(0 == strcmp(cur->module.name,module_name))
        {
            //删除
            //1.删除的是第一个节点
            if (p_msg_module_list == cur)
            {
                p_msg_module_list = cur->next;
                free(cur);
                cur = NULL;
            }
            else//删除中间节点
            {
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

void miniros_msg_module_destroy()
{
    MINIROS_MSG_MODULE_LIST_S *cur = NULL;
    MINIROS_MSG_MODULE_LIST_S *del = NULL;
//    assert(p_msg_module_list);
    cur = p_msg_module_list;
    while (cur)
    {
        del = cur;
        cur = cur->next;
        free(del);
        del = NULL;
    }
    p_msg_module_list = NULL;
}

MINIROS_MSG_MODULE_LIST_S *miniros_msg_module_find(MINIROS_MSG_MODULE_LIST_S *list_first, uint8_t *module_name)
{
    for (MINIROS_MSG_MODULE_LIST_S *cur = list_first; cur != NULL; cur = cur->next)
    {
        LOG( "Queue msg module:%s, %s!\n",cur->module.name,module_name );
        if(0 == strcmp(cur->module.name,module_name))
        {

            return cur;
        }
    }

    return NULL;
}


static void miniros_msg_task(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    MINIROS_MSG_S miniros_msg;
    BaseType_t xStatus;
    uint8_t msg_cnt = 0;
    MINIROS_MSG_MODULE_LIST_S *find_module_list;
    while(1)
    {
        msg_cnt = uxQueueMessagesWaiting( miniros_msg_queue );
        LOG( "Queue msg cnt:%d!\n",msg_cnt );

        xStatus = xQueueReceive( miniros_msg_queue, &miniros_msg, portMAX_DELAY);

        if( xStatus == pdPASS ){
            LOG( "Queue msg dest:%s,%s!\n",miniros_msg.msg_dest,miniros_msg.msg_buf );
            find_module_list = miniros_msg_module_find(p_msg_module_list,miniros_msg.msg_dest);
            if(find_module_list)
                find_module_list->module.handler(miniros_msg.msg_id,miniros_msg.msg_buf);
            else
                LOG( "Queue msg list cannot find module!\n" );
        }
        vTaskDelay(1000);
    }
}

void miniros_msg_send(uint8_t *msg_dest, uint8_t msg_id, uint8_t *msg_buf)
{
    BaseType_t xStatus;
    MINIROS_MSG_S temp_msg={0};
    memcpy(temp_msg.msg_dest,msg_dest,strlen(msg_dest)+1);
    temp_msg.msg_id = msg_id;
    memcpy(temp_msg.msg_buf,msg_buf,strlen(msg_buf)+1);
    xStatus = xQueueSendFromISR( miniros_msg_queue, &temp_msg, 0);
    if( xStatus != pdPASS )
         LOG("miniros_msg_send fail: %d\n",xStatus);

}

void miniros_msg_handle(uint8_t msg_id,uint8_t *msg_buf)
{
    ret_code_t ret;

    LOG("miniros_msg_handle:%d,%d!\n",msg_id,atoi(msg_buf));
}

void miniros_msg_init()
{

    miniros_msg_queue = xQueueCreate( MINIROS_MSG_QUEUE_MAX , sizeof( MINIROS_MSG_S) );
    xTaskCreate(miniros_msg_task,"miniros msg task",MINIROS_MSG_TASK_STACK_SIZE,NULL,MINIROS_MSG_TASK_PRIO,&miniros_msg_task_t);
    
    miniros_msg_module_register("message",miniros_msg_handle);
    LOG("miniros msg init!\n");
}



/** @} */
