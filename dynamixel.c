/*******************************************************************************
* Copyright 2018~2019 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
#  @file dynamixel.c
#  @author qqnokia@gmail.com
#******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_power.h"
#include "nrf_serial.h"
#include "app_timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stream_buffer.h"

#include "app_error.h"
#include "app_util.h"
#include "boards.h"
#include "utils.h"
#include "wifi.h"

/** @file
 * @defgroup nrf_serial_uartes_example main.c
 * @{
 * @ingroup nrf_serial_uartes_example
 * @brief Example of @ref nrf_serial usage. Loopback example using two UARTE peripherals.
 *        Please short Arduino SCL and SDA GPIOs to start transmission.
 *
 */
#define WIFI_RECV_STREAM_BUF 256
#define WIFI_RECV_STREAM_BUF_LEVEL 1

TaskHandle_t wifi_recv_task_t;
StreamBufferHandle_t wifi_recv_buf_t;

#define WIFI_UART_RX_PIN  NRF_GPIO_PIN_MAP(1,7)//8
#define WIFI_UART_TX_PIN  NRF_GPIO_PIN_MAP(1,2)//6
#define WIFI_UART_RTS_PIN  NRF_GPIO_PIN_MAP(1,4)//8
#define WIFI_UART_CTS_PIN  NRF_GPIO_PIN_MAP(1,3)//6

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER
//static void sleep_handler(void)
//{
//    LOG("sleep_handler!\n");
//    __WFE();
//    __SEV();
//    __WFE();
//    vTaskDelay(2000);
//    LOG("sleep_handler exit!\n");
//   // vTaskResume(wifi_recv_task_t);
//}

//NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte0_drv_config,
//                      RX_PIN_NUMBER, ARDUINO_SCL_PIN,
//                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
//                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
//                      NRF_UART_BAUDRATE_115200,
//                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte1_drv_config,
                      WIFI_UART_RX_PIN, WIFI_UART_TX_PIN,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);


#define SERIAL_FIFO_TX_SIZE 256
#define SERIAL_FIFO_RX_SIZE 1024

//NRF_SERIAL_QUEUES_DEF(serial0_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_QUEUES_DEF(serial1_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 200
#define SERIAL_BUFF_RX_SIZE 200

//NRF_SERIAL_BUFFERS_DEF(serial0_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial1_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);


//NRF_SERIAL_CONFIG_DEF(serial0_config, NRF_SERIAL_MODE_DMA,
//                      &serial0_queues, &serial0_buffs, NULL, sleep_handler);
NRF_SERIAL_CONFIG_DEF(serial1_config, NRF_SERIAL_MODE_DMA,
                      &serial1_queues, &serial1_buffs, NULL, NULL);


//NRF_SERIAL_UART_DEF(serial0_uarte, 0);
NRF_SERIAL_UART_DEF(serial1_uarte, 1);

//#define UART_DBG
static void wifi_recv_task(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    ret_code_t ret;
    size_t xReceivedBytes;
    uint8_t buf[WIFI_RECV_STREAM_BUF];
    while(1)
    {
        char c;
  
        ret = nrf_serial_read(&serial1_uarte, &c, sizeof(c), NULL, 0);
        if (ret != NRF_SUCCESS)
        {
            memset(buf,0,sizeof(buf));
            xReceivedBytes=xStreamBufferReceive(wifi_recv_buf_t,buf,sizeof(buf),pdMS_TO_TICKS( 100 ));
            LOG("wifi read:%d,buf:%s!\n",xReceivedBytes,buf);
            #ifdef UART_DBG
            (void)nrf_serial_write(&serial1_uarte, buf, xReceivedBytes, NULL, 0);
            (void)nrf_serial_flush(&serial1_uarte, 0);
            #endif
            vTaskDelay(1000);
            continue;
        }
        LOG("wifi read back:%c!\n",c);
        xStreamBufferSend(wifi_recv_buf_t,&c,sizeof(c),pdMS_TO_TICKS( 100 ));
       
    }
}

//static void wifi_recv_timer(void * pvParameter)
//{
//    UNUSED_PARAMETER(pvParameter);
//    ret_code_t ret;
//    char buf[100];
//    memset(buf,0,sizeof(buf));
//    int i = 0;
//    LOG("wifi_recv_timer!\n");
//    while(1)
//    {
//        char c;
//
//        ret = nrf_serial_read(&serial1_uarte, &c, sizeof(c), NULL, 0);
//        if (ret != NRF_SUCCESS)
//        {
//            LOG("wifi read:%d!\n",ret);
////            vTaskDelay(1000);
//            LOG("wifi read back:%s!\n",buf);
//            if(i>0){
//                (void)nrf_serial_write(&serial1_uarte,buf,sizeof(c)*i, NULL, 0);
//                (void)nrf_serial_flush(&serial1_uarte, 0);
//            }
//            break;
//        }
//        memcpy(buf[i++],c,sizeof(c));
//      //
//        
//    }
//}
//
//#define WIFI_RECV_TIMER_PERIOD      2000          /**< Timer period. LED1 timer will expire after 1000 ms */
void wifi_init()
{
    ret_code_t ret;
    ret = nrf_serial_init(&serial1_uarte, &m_uarte1_drv_config, &serial1_config);
    APP_ERROR_CHECK(ret);
    xTaskCreate(wifi_recv_task,"wifi recv",WIFI_RECV_TASK_STACK_SIZE,NULL,WIFI_RECV_TASK_PRIO,&wifi_recv_task_t);
    wifi_recv_buf_t = xStreamBufferCreate(WIFI_RECV_STREAM_BUF,WIFI_RECV_STREAM_BUF_LEVEL);
//    /* Start timer for wifi receiving */
//    wifi_recv_timer_t = xTimerCreate( "wifi_recv", WIFI_RECV_TIMER_PERIOD, pdTRUE, NULL, wifi_recv_timer);
//    xTimerStart(wifi_recv_timer_t, 1000);
    LOG("wifi init!\n");  

}

void wifi_cmd_send(void *cmd_buf)
{
    ret_code_t ret;
    char *cmd;
    cmd=malloc(strlen(cmd_buf)+2);
    memset(cmd,0,strlen(cmd_buf)+2);
    if(cmd) {
        memcpy(cmd,cmd_buf,strlen(cmd_buf));
        cmd[strlen(cmd)-1]='\r';
        cmd[strlen(cmd)]='\n';
        //cmd[strlen(cmd)]='\0';
        LOG("wifi_cmd_send:%s!\n",cmd);
        ret = nrf_serial_write(&serial1_uarte,
                               cmd,
                               strlen(cmd),
                               NULL,
                               NRF_SERIAL_MAX_TIMEOUT);
        (void)nrf_serial_flush(&serial1_uarte, 0);
        free(cmd);
        cmd = NULL;
    }else{
        
        LOG("wifi_cmd_send:fail!\n");
    }

}
void wifi_msg_process(uint8_t *msg_buf)
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
    LOG("wifi process:%s!\n",msg_buf);
//    ret = nrf_serial_init(&serial0_uarte, &m_uarte0_drv_config, &serial0_config);
//    APP_ERROR_CHECK(ret);


//    char tx_message[] = "AT\r\n";
    wifi_cmd_send(msg_buf);
//    ret = nrf_serial_write(&serial1_uarte,
//                           tx_message,
//                           strlen(tx_message),
//                           NULL,
//                           NRF_SERIAL_MAX_TIMEOUT);
//    (void)nrf_serial_flush(&serial1_uarte, 0);

//    wifi_recv_task(NULL); 
}

/** @} */
