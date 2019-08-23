

#ifndef __MANIPULATION_H
#define	__MANIPULATION_H

#include "nrf.h"

/******************** Mo-Tse Robot Servo Distribution **************************
*
*                           ||  ------  ||
*                           || | HEAD | ||            
*      |R_ARM_SHOULDER|=====||  ------  ||=====|L_ARM_SHOULDER|
*      |              |=====|            |=====|              |
*      |              |=====|            |=====|              |
*      |R_ARM_ELBOW   |     |============|     |   L_ARM_ELBOW|
*          |       |        |            |        |       |
*          |       |        |            |        |       |
*                           |============|
*                         |||||        |||||
*                   |R_LEG_HIP  |    |L_LEG_HIP  |
*                   |           |    |           |
*                   |           |    |           |
*                   |R_LEG_ANKLE|    |L_LEG_ANKLE|
*                   |           |    |           |
*                  |_____________|  |_____________|
*
******************************************************************************/
//Robot Servo id define
#define R_ARM_ELBOW     0
#define R_ARM_SHOULDER  1
#define R_LEG_HIP       2
#define R_LEG_ANKLE     3

#define L_ARM_ELBOW     12
#define L_ARM_SHOULDER  13
#define L_LEG_HIP       14
#define L_LEG_ANKLE     15

typedef enum manipulation_msg_e{
    R_ARM_UP = 0,
    R_ARM_DOWN,
    L_ARM_UP,
    L_ARM_DOWN,


}MANIPULATION_CMD_E;

void manipulation_module_init();

void manipulation_msg_handle(uint8_t msg_id,uint8_t *msg_buf);
#endif /* __MANIPULATION_H */






