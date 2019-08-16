#ifndef __UTILS_H
#define	__UTILS_H

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_power.h"
#include "nrf_drv_timer.h"
#include "nrf_serial.h"
#include "app_timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "stream_buffer.h"

#include "app_error.h"
#include "app_util.h"
#include "boards.h"

/* Logging and RTT */
#include "SEGGER_RTT.h"

/*define all tasks's priority and stack size*/
#define RTT_CMD_TASK_PRIO             2
#define RTT_CMD_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE+200)



#define US_MESSURE_TASK_PRIO          1
#define US_MESSURE_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE+100)

#define WIFI_RECV_TASK_PRIO           1
#define WIFI_RECV_TASK_STACK_SIZE           (configMINIMAL_STACK_SIZE+100)

#define LOBOT_RECV_TASK_PRIO          1
#define LOBOT_RECV_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE+100)

/* Logging and RTT */
#define LOG(str, ...) SEGGER_RTT_printf(0, RTT_CTRL_RESET str, ##__VA_ARGS__)
 
#endif /* __UTILS_H */






