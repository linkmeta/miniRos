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
#include "miniros.h"
#include "sensor.h"
void sensor_msg_handle(uint8_t msg_id,uint8_t *msg_buf)
{
    ret_code_t ret;
    int distance = 0;
    char buf[20];
    LOG("sensor_msg_handle:%d,%s!\n",msg_id,msg_buf);
    switch(msg_id){

        case DISTANCE_INFO:
        distance = atoi(msg_buf);
        sprintf(buf,"distance:%4dcm",distance/10);
        miniros_cmd_send("oled",1,buf);
        miniros_cmd_send("bt",0,buf);
        break;

        default:
        break;

    }
}

void sensor_module_init()
{
    miniros_msg_module_register("sensor",sensor_msg_handle);
    LOG("sensor module init!\n");
}
