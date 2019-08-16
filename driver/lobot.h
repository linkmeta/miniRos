

#ifndef __LOBOT_H
#define	__LOBOT_H

#include "nrf.h"

#define FRAME_HEADER 0x55             //֡ͷ
#define CMD_SERVO_MOVE 0x03           //����ƶ�ָ��
#define CMD_ACTION_GROUP_RUN 0x06     //���ж�����ָ��
#define CMD_ACTION_GROUP_STOP 0x07    //ֹͣ������ָ��
#define CMD_ACTION_GROUP_SPEED 0x0B   //���ö����������ٶ�
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //��ȡ��ص�ѹָ��

typedef enum lobot_cmd_e{
    GET_BATTERY_VOLTAGE = 0,
    SINGLE_CONTROL,
    GROUP_CONTROL,
    STOP_CONTROL,

}LOBOT_CMD_E;

typedef struct _lobot_servo_ {  //���ID,���Ŀ��λ��
	uint8_t ID;
	uint16_t Position;
} LobotServo;

void lobot_init();

void lobot_cmd_process(uint8_t cmd_id, uint8_t *cmd_buf);

//
//extern bool isUartRxCompleted;
//extern uint8_t LobotRxBuf[16];
//extern uint16_t batteryVolt;
//extern void receiveHandle(void);

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);
void getBatteryVoltage(void);

#endif /* __LOBOT_H */