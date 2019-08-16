

#ifndef __UTILS_H
#define	__UTILS_H


/*define all tasks's priority and stack size*/
#define RTT_CMD_TASK_PRIO             2
#define RTT_CMD_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE+100)

#define ROBOT_PROCESS_TASK_PRIO       2
#define ROBOT_PROCESS_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE+200)

#define US_MESSURE_TASK_PRIO          1
#define US_MESSURE_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE+100)

#define WIFI_RECV_TASK_PRIO           1
#define WIFI_RECV_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE+100)

#define LOBOT_RECV_TASK_PRIO          1
#define LOBOT_RECV_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE+100)

/* Logging and RTT */
#include "SEGGER_RTT.h"

#define LOG(str, ...) SEGGER_RTT_printf(0, RTT_CTRL_RESET str, ##__VA_ARGS__)
 
#endif /* __UTILS_H */






