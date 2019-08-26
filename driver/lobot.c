/*******************************************************************************
* Copyright 2018~2019 
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
#  @file lobot.c
#  @brief lobot sub-board for servo control.
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
#include "miniRos.h"
#define LOBOT_RECV_STREAM_BUF 256
#define LOBOT_RECV_STREAM_BUF_LEVEL 1


#define LOBOT_UART_RX_PIN  8//NRF_GPIO_PIN_MAP(1,7)//8
#define LOBOT_UART_TX_PIN  6//NRF_GPIO_PIN_MAP(1,2)//6
#define LOBOT_UART_RTS_PIN  5//NRF_GPIO_PIN_MAP(1,4)//8
#define LOBOT_UART_CTS_PIN  7//NRF_GPIO_PIN_MAP(1,3)//6


#define SERIAL_FIFO_TX_SIZE 256
#define SERIAL_FIFO_RX_SIZE 1024
#define SERIAL_BUFF_TX_SIZE 200
#define SERIAL_BUFF_RX_SIZE 200

#define GET_LOW_BYTE(A) ((uint8_t)(A))

#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

TaskHandle_t lobot_recv_task_t;
StreamBufferHandle_t lobot_recv_buf_t;


NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte0_drv_config,
                      LOBOT_UART_RX_PIN, LOBOT_UART_TX_PIN,
                      LOBOT_UART_RTS_PIN, LOBOT_UART_CTS_PIN,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_9600,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);



//NRF_SERIAL_QUEUES_DEF(serial0_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_QUEUES_DEF(serial0_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);



//NRF_SERIAL_BUFFERS_DEF(serial0_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial0_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);


//NRF_SERIAL_CONFIG_DEF(serial0_config, NRF_SERIAL_MODE_DMA,
//                      &serial0_queues, &serial0_buffs, NULL, sleep_handler);
NRF_SERIAL_CONFIG_DEF(serial0_config, NRF_SERIAL_MODE_DMA,
                      &serial0_queues, &serial0_buffs, NULL, NULL);


NRF_SERIAL_UART_DEF(serial0_uarte, 0);

//extern bool isUartRxCompleted;

uint8_t LobotTxBuf[128];  //发送缓存
uint8_t LobotRxBuf[16];
uint16_t batteryVolt;

void lobot_uart_write(void *buf, uint32_t len)
{
    LOG("lobot_uart_write:%s!\n",buf);
    nrf_serial_write(&serial0_uarte,
                           buf,
                           len,
                           NULL,
                           NRF_SERIAL_MAX_TIMEOUT);
    (void)nrf_serial_flush(&serial0_uarte, 0);
}

/*********************************************************************************
 * Function:  moveServo
 * Description： 控制单个舵机转动
 * Parameters:   sevoID:舵机ID，Position:目标位置,Time:转动时间
                    舵机ID取值:0<=舵机ID<=31,Time取值: Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
    LOG("moveServo:%d %d %d\n",servoID,Position,Time);
    if (servoID > 31 || !(Time > 0)) {  //舵机ID不能打于31,可根据对应控制板修改
       return;
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;    //填充帧头
    LobotTxBuf[2] = 8;
    LobotTxBuf[3] = CMD_SERVO_MOVE;           //数据长度=要控制舵机数*3+5，此处=1*3+5//填充舵机移动指令
    LobotTxBuf[4] = 1;                        //要控制的舵机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);       //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);      //取得时间的高八位
    LobotTxBuf[7] = servoID;                  //舵机ID
    LobotTxBuf[8] = GET_LOW_BYTE(Position);   //取得目标位置的低八位
    LobotTxBuf[9] = GET_HIGH_BYTE(Position);  //取得目标位置的高八位

    lobot_uart_write(LobotTxBuf, 10);
    LOG("lobot moveServo:%d,%d,%d!\n",servoID,Position,Time);
}

/*********************************************************************************
 * Function:  moveServosByArray
 * Description： 控制多个舵机转动
 * Parameters:   servos[]:舵机结体数组，Num:舵机个数,Time:转动时间
                    0 < Num <= 32,Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
    uint8_t index = 7;
    uint8_t i = 0;

    if (Num < 1 || Num > 32 || !(Time > 0)) {
        return;                                          //舵机数不能为零和大与32，时间不能为零
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                       //数据长度 = 要控制舵机数*3+5
    LobotTxBuf[3] = CMD_SERVO_MOVE;                    //填充舵机移动指令
    LobotTxBuf[4] = Num;                               //要控制的舵机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);                //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);               //取得时间的高八位

    for (i = 0; i < Num; i++) {                        //循环填充舵机ID和对应目标位置
        LobotTxBuf[index++] = servos[i].ID;              //填充舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position); //填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position);//填充目标位置高八位
    }

    lobot_uart_write(LobotTxBuf, LobotTxBuf[2] + 2);             //发送


}

/*********************************************************************************
 * Function:  moveServos
 * Description： 控制多个舵机转动
 * Parameters:   Num:舵机个数,Time:转动时间,...:舵机ID,转动角，舵机ID,转动角度 如此类推
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServos(uint8_t Num, uint16_t Time, ...)
{
    uint8_t index = 7;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;  //

    va_start(arg_ptr, Time); //取得可变参数首地址
    if (Num < 1 || Num > 32) {
        return;               //舵机数不能为零和大与32，时间不能小于0
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                //数据长度 = 要控制舵机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_MOVE;             //舵机移动指令
    LobotTxBuf[4] = Num;                        //要控制舵机数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);         //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);        //取得时间的高八位

    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
        temp = va_arg(arg_ptr, int);  //可变参数中取得对应目标位置
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); //填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(temp);//填充目标位置高八位
    }

    va_end(arg_ptr);  //置空arg_ptr

    lobot_uart_write(LobotTxBuf, LobotTxBuf[2] + 2);    //发送
    LOG("lobot moveServos:%d,%d!\n",Num,Time);
}


/*********************************************************************************
 * Function:  runActionGroup
 * Description： 运行指定动作组
 * Parameters:   NumOfAction:动作组序号, Times:执行次数
 * Return:       无返回
 * Others:       Times = 0 时无限循环
 **********************************************************************************/
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;  //填充帧头
    LobotTxBuf[2] = 5;                      //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
    LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;   //填充运行动作组命令
    LobotTxBuf[4] = numOfAction;            //填充要运行的动作组号
    LobotTxBuf[5] = GET_LOW_BYTE(Times);    //取得要运行次数的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Times);   //取得要运行次数的高八位

    lobot_uart_write(LobotTxBuf, 7);            //发送
    
    LOG("lobot runActionGroup:%d,%d!\n",numOfAction,Times);

}

/*********************************************************************************
 * Function:  stopActiongGroup
 * Description： 停止动作组运行
 * Parameters:   Speed: 目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void stopActionGroup(void)
{
    LobotTxBuf[0] = FRAME_HEADER;     //填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;                //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
    LobotTxBuf[3] = CMD_ACTION_GROUP_STOP;   //填充停止运行动作组命令

    lobot_uart_write(LobotTxBuf, 4);      //发送
    LOG("lobot stopActionGroup!\n");
}
/*********************************************************************************
 * Function:  setActionGroupSpeed
 * Description： 设定指定动作组的运行速度
 * Parameters:   NumOfAction: 动作组序号 , Speed:目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   //填充帧头
    LobotTxBuf[2] = 5;                       //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
    LobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;  //填充设置动作组速度命令
    LobotTxBuf[4] = numOfAction;             //填充要设置的动作组号
    LobotTxBuf[5] = GET_LOW_BYTE(Speed);     //获得目标速度的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Speed);    //获得目标熟读的高八位

    lobot_uart_write(LobotTxBuf, 7);             //发送
}

/*********************************************************************************
 * Function:  setAllActionGroupSpeed
 * Description： 设置所有动作组的运行速度
 * Parameters:   Speed: 目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void setAllActionGroupSpeed(uint16_t Speed)
{
    setActionGroupSpeed(0xFF, Speed);  //调用动作组速度设定，组号为0xFF时设置所有组的速度
}

/*********************************************************************************
 * Function:  getBatteryVoltage
 * Description： 发送获取电池电压命令
 * Parameters:   Timeout：重试次数
 * Return:       无返回
 * Others:
 **********************************************************************************/
void getBatteryVoltage(void)
{
//	uint16_t Voltage = 0;
    LobotTxBuf[0] = FRAME_HEADER;  //填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;             //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
    LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  //填充获取电池电压命令

    lobot_uart_write(LobotTxBuf, 4);   //发送

}

void recv_handler(char *buf)
{
    char temp[30];
    switch (buf[3]) {
    case CMD_GET_BATTERY_VOLTAGE: //获取电压
        batteryVolt = (((uint16_t)(buf[5])) << 8) | (buf[4]);
        LOG("lobot get battery voltage:%d!\n",batteryVolt);

        sprintf(temp,"battery volt:%dmV",batteryVolt);
//        oled_show_info(2,temp);
//        bt_send_data(temp,strlen(temp));
        miniros_cmd_send("oled",2,temp);
        miniros_cmd_send("bt",0,temp);

        break;
    default:
        break;
    }
}
//
//extern void bt_send_data( uint8_t   * p_data,
//                   uint16_t  length);
static void lobot_recv_task(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    ret_code_t ret;
    size_t xReceivedBytes;
    uint8_t buf[20];
    while(1)
    {
        char c;
  
        ret = nrf_serial_read(&serial0_uarte, &c, sizeof(c), NULL, 0);
        if (ret != NRF_SUCCESS)
        {
            memset(buf,0,sizeof(buf));
            xReceivedBytes=xStreamBufferReceive(lobot_recv_buf_t,buf,sizeof(buf),pdMS_TO_TICKS( 10 ));
            LOG("lobot read:%d,buf:%s!\n",xReceivedBytes,buf);
            #ifdef UART_DBG
            (void)nrf_serial_write(&serial0_uarte, buf, xReceivedBytes, NULL, 0);
            (void)nrf_serial_flush(&serial0_uarte, 0);
            #endif
            if(xReceivedBytes>0)
                recv_handler(buf);
            vTaskDelay(1000);
            
            continue;
        }
        LOG("lobot read back:%x!\n",c);
        xStreamBufferSend(lobot_recv_buf_t,&c,sizeof(c),pdMS_TO_TICKS( 10 ));
       
    }
}



void lobot_cmd_send(void *cmd_buf)
{
    ret_code_t ret;
    char *cmd;
    cmd=malloc(strlen(cmd_buf)+2);
    memset(cmd,0,strlen(cmd_buf)+2);
    if (cmd) {
        memcpy(cmd,cmd_buf,strlen(cmd_buf));
        cmd[strlen(cmd)-1]='\r';
        cmd[strlen(cmd)]='\n';
        //cmd[strlen(cmd)]='\0';
        LOG("lobot_cmd_send:%s!\n",cmd);
        ret = nrf_serial_write(&serial0_uarte,
                               cmd,
                               strlen(cmd),
                               NULL,
                               NRF_SERIAL_MAX_TIMEOUT);
        (void)nrf_serial_flush(&serial0_uarte, 0);
        free(cmd);
        cmd = NULL;
    } else {
        LOG("lobot_cmd_send:fail!\n");
    }
}
void lobot_cmd_process(uint8_t cmd_id, uint8_t *cmd_buf)
{
    ret_code_t ret;
    int id;
    int position;
    int time;
//    uint8_t buf[100];
//    uint8_t buf1[10],buf2[10],buf3[10];
    LOG("lobot cmd process:%d,%s!\n",cmd_id,cmd_buf);
//    ret = nrf_serial_init(&serial0_uarte, &m_uarte0_drv_config, &serial0_config);
//    APP_ERROR_CHECK(ret);
//    memset(buf,0,sizeof(buf));
//    memset(buf1,0,sizeof(buf1));
//    memset(buf2,0,sizeof(buf2));
//    memset(buf3,0,sizeof(buf3));
//    memcpy(buf,cmd_buf,strlen(cmd_buf));
    switch(cmd_id)
    {
        case GET_BATTERY_VOLTAGE:
        LOG("lobot get battery voltage!\n");
        getBatteryVoltage();
        break;
        case SINGLE_CONTROL:
            LOG("lobot single process:%s!\n",cmd_buf);
//            sscanf(buf,"%[0-9]-%[0-9]-%[0-9]",buf1,buf2,buf3);
            sscanf(cmd_buf,"%d-%d-%d",&id,&position,&time);
            LOG("lobot single process:%d %d %d!\n",id,position,time);
//            LOG("lobot single process:%s %s %s!\n",buf1,buf2,buf3);
//            id = atoi(buf1);
//            position = atoi(buf2);
//            time = atoi(buf3);
//            moveServo(1,1500,500);
            moveServo(id,position,time);
        break;

        case GROUP_CONTROL:
            LOG("lobot group process:%s!\n",cmd_buf);
//            sscanf(cmd_buf,"%s-%s",&buf1,&buf2);
            sscanf(cmd_buf,"%d-%d",&id,&time);
//            id = atoi(buf1);
//            time = atoi(buf2);
            runActionGroup(id,time);
        break;
        case STOP_CONTROL:
            stopActionGroup();
        break;
        default:
        break;

    
    }
    #if 0
    if (strlen(cmd_buf)>0) {
        if (strstr(cmd_buf,"b")){
            getBatteryVoltage();
        } else if (strstr(cmd_buf,"o")){
            buf = cmd_buf + strlen("o");
            LOG("lobot single process:%s!\n",buf);
            sscanf(buf,"%s %s %s",&buf1,&buf2,&buf3);
            id = atoi(buf1);
            position = atoi(buf2);
            time = atoi(buf3);
           // LOG("lobot board process:%s!id=%d,position=%d,time=%d\n",buf,atoi(buf1),atoi(buf2),atoi(buf3));
            moveServo(id,position,time);
        } else if (strstr(cmd_buf,"g")){
            buf = cmd_buf + strlen("g");
            LOG("lobot group process:%s!\n",buf);
            sscanf(buf,"%s %s",&buf1,&buf2);
            id = atoi(buf1);
            time = atoi(buf2);
            runActionGroup(id,time);
        } else if (strstr(cmd_buf,"e")){
            stopActionGroup();
        
        } else if (strstr(cmd_buf,"m")){
         
        }
    }
    #endif

}

void lobot_init()
{
    ret_code_t ret;
    ret = nrf_serial_init(&serial0_uarte, &m_uarte0_drv_config, &serial0_config);
    APP_ERROR_CHECK(ret);
    xTaskCreate(lobot_recv_task,"lobot recv",LOBOT_RECV_TASK_STACK_SIZE,NULL,LOBOT_RECV_TASK_PRIO,&lobot_recv_task_t);
    lobot_recv_buf_t = xStreamBufferCreate(LOBOT_RECV_STREAM_BUF,LOBOT_RECV_STREAM_BUF_LEVEL);
    
    miniros_dev_register("lobot",lobot_cmd_process);
    LOG("lobot board init!\n");  
}
/** @} */
