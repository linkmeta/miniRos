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

uint8_t LobotTxBuf[128];  //���ͻ���
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
 * Description�� ���Ƶ������ת��
 * Parameters:   sevoID:���ID��Position:Ŀ��λ��,Time:ת��ʱ��
                    ���IDȡֵ:0<=���ID<=31,Timeȡֵ: Time > 0
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
    LOG("moveServo:%d %d %d\n",servoID,Position,Time);
    if (servoID > 31 || !(Time > 0)) {  //���ID���ܴ���31,�ɸ��ݶ�Ӧ���ư��޸�
       return;
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;    //���֡ͷ
    LobotTxBuf[2] = 8;
    LobotTxBuf[3] = CMD_SERVO_MOVE;           //���ݳ���=Ҫ���ƶ����*3+5���˴�=1*3+5//������ƶ�ָ��
    LobotTxBuf[4] = 1;                        //Ҫ���ƵĶ������
    LobotTxBuf[5] = GET_LOW_BYTE(Time);       //ȡ��ʱ��ĵͰ�λ
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);      //ȡ��ʱ��ĸ߰�λ
    LobotTxBuf[7] = servoID;                  //���ID
    LobotTxBuf[8] = GET_LOW_BYTE(Position);   //ȡ��Ŀ��λ�õĵͰ�λ
    LobotTxBuf[9] = GET_HIGH_BYTE(Position);  //ȡ��Ŀ��λ�õĸ߰�λ

    lobot_uart_write(LobotTxBuf, 10);
    LOG("lobot moveServo:%d,%d,%d!\n",servoID,Position,Time);
}

/*********************************************************************************
 * Function:  moveServosByArray
 * Description�� ���ƶ�����ת��
 * Parameters:   servos[]:����������飬Num:�������,Time:ת��ʱ��
                    0 < Num <= 32,Time > 0
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
    uint8_t index = 7;
    uint8_t i = 0;

    if (Num < 1 || Num > 32 || !(Time > 0)) {
        return;                                          //���������Ϊ��ʹ���32��ʱ�䲻��Ϊ��
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //���֡ͷ
    LobotTxBuf[2] = Num * 3 + 5;                       //���ݳ��� = Ҫ���ƶ����*3+5
    LobotTxBuf[3] = CMD_SERVO_MOVE;                    //������ƶ�ָ��
    LobotTxBuf[4] = Num;                               //Ҫ���ƵĶ������
    LobotTxBuf[5] = GET_LOW_BYTE(Time);                //ȡ��ʱ��ĵͰ�λ
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);               //ȡ��ʱ��ĸ߰�λ

    for (i = 0; i < Num; i++) {                        //ѭ�������ID�Ͷ�ӦĿ��λ��
        LobotTxBuf[index++] = servos[i].ID;              //�����ID
        LobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position); //���Ŀ��λ�õͰ�λ
        LobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position);//���Ŀ��λ�ø߰�λ
    }

    lobot_uart_write(LobotTxBuf, LobotTxBuf[2] + 2);             //����


}

/*********************************************************************************
 * Function:  moveServos
 * Description�� ���ƶ�����ת��
 * Parameters:   Num:�������,Time:ת��ʱ��,...:���ID,ת���ǣ����ID,ת���Ƕ� �������
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void moveServos(uint8_t Num, uint16_t Time, ...)
{
    uint8_t index = 7;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;  //

    va_start(arg_ptr, Time); //ȡ�ÿɱ�����׵�ַ
    if (Num < 1 || Num > 32) {
        return;               //���������Ϊ��ʹ���32��ʱ�䲻��С��0
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //���֡ͷ
    LobotTxBuf[2] = Num * 3 + 5;                //���ݳ��� = Ҫ���ƶ���� * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_MOVE;             //����ƶ�ָ��
    LobotTxBuf[4] = Num;                        //Ҫ���ƶ����
    LobotTxBuf[5] = GET_LOW_BYTE(Time);         //ȡ��ʱ��ĵͰ�λ
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);        //ȡ��ʱ��ĸ߰�λ

    for (i = 0; i < Num; i++) {//�ӿɱ������ȡ�ò�ѭ�������ID�Ͷ�ӦĿ��λ��
        temp = va_arg(arg_ptr, int);//�ɲ�����ȡ�ö��ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
        temp = va_arg(arg_ptr, int);  //�ɱ������ȡ�ö�ӦĿ��λ��
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); //���Ŀ��λ�õͰ�λ
        LobotTxBuf[index++] = GET_HIGH_BYTE(temp);//���Ŀ��λ�ø߰�λ
    }

    va_end(arg_ptr);  //�ÿ�arg_ptr

    lobot_uart_write(LobotTxBuf, LobotTxBuf[2] + 2);    //����
    LOG("lobot moveServos:%d,%d!\n",Num,Time);
}


/*********************************************************************************
 * Function:  runActionGroup
 * Description�� ����ָ��������
 * Parameters:   NumOfAction:���������, Times:ִ�д���
 * Return:       �޷���
 * Others:       Times = 0 ʱ����ѭ��
 **********************************************************************************/
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;  //���֡ͷ
    LobotTxBuf[2] = 5;                      //���ݳ��ȣ�����֡��֡ͷ���������ֽ�����������̶�Ϊ5
    LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;   //������ж���������
    LobotTxBuf[4] = numOfAction;            //���Ҫ���еĶ������
    LobotTxBuf[5] = GET_LOW_BYTE(Times);    //ȡ��Ҫ���д����ĵͰ�λ
    LobotTxBuf[6] = GET_HIGH_BYTE(Times);   //ȡ��Ҫ���д����ĸ߰�λ

    lobot_uart_write(LobotTxBuf, 7);            //����
    
    LOG("lobot runActionGroup:%d,%d!\n",numOfAction,Times);

}

/*********************************************************************************
 * Function:  stopActiongGroup
 * Description�� ֹͣ����������
 * Parameters:   Speed: Ŀ���ٶ�
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void stopActionGroup(void)
{
    LobotTxBuf[0] = FRAME_HEADER;     //���֡ͷ
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;                //���ݳ��ȣ�����֡��֡ͷ���������ֽ�����������̶�Ϊ2
    LobotTxBuf[3] = CMD_ACTION_GROUP_STOP;   //���ֹͣ���ж���������

    lobot_uart_write(LobotTxBuf, 4);      //����
    LOG("lobot stopActionGroup!\n");
}
/*********************************************************************************
 * Function:  setActionGroupSpeed
 * Description�� �趨ָ��������������ٶ�
 * Parameters:   NumOfAction: ��������� , Speed:Ŀ���ٶ�
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   //���֡ͷ
    LobotTxBuf[2] = 5;                       //���ݳ��ȣ�����֡��֡ͷ���������ֽ�����������̶�Ϊ5
    LobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;  //������ö������ٶ�����
    LobotTxBuf[4] = numOfAction;             //���Ҫ���õĶ������
    LobotTxBuf[5] = GET_LOW_BYTE(Speed);     //���Ŀ���ٶȵĵͰ�λ
    LobotTxBuf[6] = GET_HIGH_BYTE(Speed);    //���Ŀ������ĸ߰�λ

    lobot_uart_write(LobotTxBuf, 7);             //����
}

/*********************************************************************************
 * Function:  setAllActionGroupSpeed
 * Description�� �������ж�����������ٶ�
 * Parameters:   Speed: Ŀ���ٶ�
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void setAllActionGroupSpeed(uint16_t Speed)
{
    setActionGroupSpeed(0xFF, Speed);  //���ö������ٶ��趨�����Ϊ0xFFʱ������������ٶ�
}

/*********************************************************************************
 * Function:  getBatteryVoltage
 * Description�� ���ͻ�ȡ��ص�ѹ����
 * Parameters:   Timeout�����Դ���
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void getBatteryVoltage(void)
{
//	uint16_t Voltage = 0;
    LobotTxBuf[0] = FRAME_HEADER;  //���֡ͷ
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;             //���ݳ��ȣ�����֡��֡ͷ���������ֽ�����������̶�Ϊ2
    LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  //����ȡ��ص�ѹ����

    lobot_uart_write(LobotTxBuf, 4);   //����

}

void recv_handler(char *buf)
{
    char temp[30];
    switch (buf[3]) {
    case CMD_GET_BATTERY_VOLTAGE: //��ȡ��ѹ
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
