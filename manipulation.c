/*******************************************************************************
* Copyright 2018~2019 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
#  @file manipulation.c
#  @brief 
#  @author qqnokia@gmail.com
#******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>

#include "utils.h"
#include "lobot.h"
#include "oled.h"
#include "nrf52840.h"
#include "miniros.h"
#include "message.h"
#include "manipulation.h"

void manipulation_msg_handle(uint8_t msg_id,uint8_t *msg_buf)
{
    ret_code_t ret;
    int distance = 0;
    char buf[20];
    LOG("manipulation_msg_handle:%d,%s!\n",msg_id,msg_buf);
    switch(msg_id){

        case R_ARM_UP:
        distance = atoi(msg_buf);
        sprintf(buf,"%d-2000-1000",R_ARM_SHOULDER);
        miniros_cmd_send("oled",7,"right arm up");
        miniros_cmd_send("lobot",SINGLE_CONTROL,buf);
        break;
        
        case R_ARM_DOWN:
        
        break;
        
        case L_ARM_UP:
        
        break;
        
        case L_ARM_DOWN:

        break;
        default:
        break;

    }

}

void manipulation_module_init()
{
    miniros_msg_module_register("manipulation",manipulation_msg_handle);
    LOG("manipulation module init!\n");
}
/** @} */
