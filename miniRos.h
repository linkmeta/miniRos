
#ifndef __MINIROS_H
#define	__MINIROS_H

#include <stdint.h>
#include <string.h>
#include "utils.h"

#define MINIROS_PROCESS_TASK_PRIO             2
#define MINIROS_PROCESS_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE+400)

#define MINIROS_CMD_QUEUE_MAX 10

typedef struct miniros_cmd_s{
  uint8_t cmd_id;
  uint8_t dev[20];
  uint8_t cmd_buf[32];
}MINIROS_CMD_S;


typedef struct miniros_device_s{
  uint8_t name[20];
  void (*handler)(uint8_t cmd_id,uint8_t *cmd_buf);
}MINIROS_DEVICE_S;

typedef struct miniros_dev_list_s{
    MINIROS_DEVICE_S device;
    struct miniros_dev_list_s *next;

}MINIROS_DEV_LIST_S;

#if 0
typedef struct miniros_cmd_s{
  uint8_t cmd_id;
  uint8_t cmd_buf[32];

}MINIROS_CMD_S;

#define MINIROS_CMD_QUEUE_MAX 10
#define MINIROS_DEVICE_NUM_MAX 10

typedef enum miniros_device_e{
    WIFI = 0,
    LED,
    OLED,
    US,
    LOBOT,
    BT,

}MINIROS_DEVICE_E;

static uint8_t miniros_device_idx[MINIROS_DEVICE_NUM_MAX][32] = {
    "wifi:",
    "led:",
    "lcd:",
    "us:",
    "bot:",
    "bt:",

};

#endif
void miniros_init();
void miniros_cmd_send(uint8_t *dev, uint8_t cmd_id, uint8_t *cmd_buf);
void miniros_dev_register(uint8_t *dev_name,void (*handler)(uint8_t cmd_id,uint8_t *cmd_buf));
#endif /* __MINIROS_H */






