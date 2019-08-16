/*******************************************************************************
* Copyright 2018~2019 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
#  @file us_026.c
#  @brief ultrasonic module driver for us026.
#  @author qqnokia@gmail.com
#******************************************************************************/

#include "us_026.h"
#include "oled.h"
#include "utils.h"

const nrf_drv_timer_t TIMER_US026 = NRF_DRV_TIMER_INSTANCE(1);

#define US026_TRIG_PIN        NRF_GPIO_PIN_MAP(1,8)
#define US026_ECHO_PIN        NRF_GPIO_PIN_MAP(1,9)
#define US026_ECHO_TIMEOUT    600000
#define US026_MESSURE_TIMER_PERIOD 2000

TimerHandle_t us026_messure_timer_handle;
TaskHandle_t us_task_t;

int g_cur_distance = 0;

int us026_get_distance()
{
    return g_cur_distance;

}
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    ;
}

static void us_messure_task(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
//    ret_code_t ret;
    int distance = 0;
    char buf[20];
//    size_t xReceivedBytes;
//    uint8_t buf[WIFI_RECV_STREAM_BUF];
    while(1)
    {
        us026_messure();
        vTaskDelay(US026_MESSURE_TIMER_PERIOD); 
        LOG("us get distance:%d!\n",g_cur_distance);
//        memset(buf,0,sizeof(buf));
        sprintf(buf,"distance:%4dcm",g_cur_distance/10);
//        oled_show_info1(buf);
        miniros_cmd_send("oled",1,buf);
      
    }
}

int64_t cnt,cnt_new;
void irq_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//    nrf_drv_gpiote_out_toggle(LED_1);
    uint8_t string[10];
    int gpio_val = nrf_gpio_pin_read(US026_ECHO_PIN);
    if(gpio_val==1){
        cnt_new = nrfx_timer_capture(&TIMER_US026,NRF_TIMER_CC_CHANNEL0);
        LOG("us026 high begin::cnt_new=%d!\n",cnt_new);
        if((cnt_new-cnt)>US026_ECHO_TIMEOUT)
        {
            LOG("us026 messure too far!\n");
            nrf_drv_timer_disable(&TIMER_US026);
            return;
        }
        cnt = cnt_new;
    }else{
        cnt_new = nrfx_timer_capture(&TIMER_US026,NRF_TIMER_CC_CHANNEL0);
        LOG("us026 high over ::cnt_new=%d!\n",cnt_new);
        if((cnt_new-cnt)>US026_ECHO_TIMEOUT)
        {
            LOG("us026_messure too far!\n");
            nrf_drv_timer_disable(&TIMER_US026);
            return;
        }
        nrf_drv_timer_disable(&TIMER_US026);
        g_cur_distance = (cnt_new-cnt)*17/1600;//mm
        LOG("us026 messure distance is:%dmm\n",g_cur_distance);
        itoa(g_cur_distance,string,10);
        miniros_msg_send("sensor",0,string);
//        miniros_msg_send("message",0,string);
    }
}

void us026_init(void)
{
    ret_code_t err_code;
//    err_code = nrf_drv_gpiote_init();
//    APP_ERROR_CHECK(err_code);
    nrf_gpio_cfg_output(US026_TRIG_PIN);

    nrf_drv_gpiote_in_config_t irq_pin_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    irq_pin_config.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrf_drv_gpiote_in_init(US026_ECHO_PIN, &irq_pin_config, irq_pin_handler);
    nrf_drv_gpiote_in_event_enable(US026_ECHO_PIN, true);
//    nrf_gpio_cfg_input(US026_ECHO_PIN,GPIO_PIN_CNF_PULL_Disabled);

    nrf_drv_timer_config_t timer_cfg = {
    .frequency          = 0,
    .mode               = 0,
    .bit_width          = 3,
    .interrupt_priority = 6,
    .p_context          = NULL
    };
    err_code = nrf_drv_timer_init(&TIMER_US026, &timer_cfg, timer_event_handler);

    APP_ERROR_CHECK(err_code);
    xTaskCreate(us_messure_task,"us messure",US_MESSURE_TASK_STACK_SIZE,NULL,US_MESSURE_TASK_PRIO,us_task_t);
}

void us026_triger(void)
{
    nrf_gpio_pin_write(US026_TRIG_PIN,1);
    nrf_delay_us(20);
    nrf_gpio_pin_write(US026_TRIG_PIN,0);
}

void us026_messure(void)
{
    nrf_drv_timer_enable(&TIMER_US026);
    cnt = nrfx_timer_capture(&TIMER_US026,NRF_TIMER_CC_CHANNEL0);
    LOG("us026_messure::cnt=%d!\n",cnt);
    us026_triger();
}

