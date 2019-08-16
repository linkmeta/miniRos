/*******************************************************************************
* Copyright 2018~2019 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
#  @file wifi.c
#  @brief esp8266 wifi module driver.
#  @author qqnokia@gmail.com
#******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "utils.h"
#include "wifi.h"
#include "miniRos.h"
#define WIFI_RECV_STREAM_BUF 256
#define WIFI_RECV_STREAM_BUF_LEVEL 1

#define WIFI_UART_RX_PIN  NRF_GPIO_PIN_MAP(1,7)//8
#define WIFI_UART_TX_PIN  NRF_GPIO_PIN_MAP(1,2)//6
#define WIFI_UART_RTS_PIN  NRF_GPIO_PIN_MAP(1,4)//8
#define WIFI_UART_CTS_PIN  NRF_GPIO_PIN_MAP(1,3)//6


#define SERIAL_FIFO_TX_SIZE 256
#define SERIAL_FIFO_RX_SIZE 1024
#define SERIAL_BUFF_TX_SIZE 200
#define SERIAL_BUFF_RX_SIZE 200

TaskHandle_t wifi_recv_task_t;
StreamBufferHandle_t wifi_recv_buf_t;

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte1_drv_config,
                      WIFI_UART_RX_PIN, WIFI_UART_TX_PIN,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

//NRF_SERIAL_QUEUES_DEF(serial0_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_QUEUES_DEF(serial1_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);

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
//        LOG("wifi read back:%c!\n",c);
        xStreamBufferSend(wifi_recv_buf_t,&c,sizeof(c),pdMS_TO_TICKS( 100 ));
       
    }
}

void wifi_cmd_send(uint8_t *cmd_buf)
{
    ret_code_t ret;
    uint8_t *cmd;
    uint8_t cmd_len;
    LOG("wifi cmd send:%s %d!\n",cmd_buf,strlen(cmd_buf));
    cmd_len=strlen(cmd_buf)+2;
    cmd=malloc(cmd_len);
    if(cmd) {
        memset(cmd,0,cmd_len);
        memcpy(cmd,cmd_buf,strlen(cmd_buf));
        cmd[cmd_len-2]='\r';
        cmd[cmd_len-1]='\n';
        cmd[cmd_len]='\0';
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
void wifi_cmd_process(uint8_t cmd_id,uint8_t *cmd_buf)
{
    ret_code_t ret;

    LOG("wifi cmd process:%s!\n",cmd_buf);
    wifi_cmd_send(cmd_buf);
}


void wifi_init()
{
    ret_code_t ret;
    ret = nrf_serial_init(&serial1_uarte, &m_uarte1_drv_config, &serial1_config);
    APP_ERROR_CHECK(ret);
    xTaskCreate(wifi_recv_task,"wifi recv",WIFI_RECV_TASK_STACK_SIZE,NULL,WIFI_RECV_TASK_PRIO,&wifi_recv_task_t);
    wifi_recv_buf_t = xStreamBufferCreate(WIFI_RECV_STREAM_BUF,WIFI_RECV_STREAM_BUF_LEVEL);

    miniros_dev_register("wifi",wifi_cmd_process);
    LOG("wifi init!\n");
}
/** @} */
