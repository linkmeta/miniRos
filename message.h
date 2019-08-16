

#ifndef __MESSAGE_H
#define	__MESSAGE_H


#define MINIROS_MSG_TASK_PRIO       2
#define MINIROS_MSG_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE+200)

#define MINIROS_MSG_QUEUE_MAX 10
//#define MINIROS_MODULE_NUM_MAX 10

typedef struct miniros_msg_s{
  uint8_t msg_id;
  uint8_t msg_dest[20];
  uint8_t msg_buf[32];
}MINIROS_MSG_S;


typedef struct miniros_module_s{
  uint8_t name[20];
  void (*handler)(uint8_t msg_id,uint8_t *msg_buf);
}MINIROS_MODULE_S;

typedef struct miniros_msg_module_list_s{
    MINIROS_MODULE_S module;
    struct miniros_msg_module_list_s *next;

}MINIROS_MSG_MODULE_LIST_S;

//static uint8_t miniros_module_idx[MINIROS_MODULE_NUM_MAX][20] = {
//    "sensor",
//    "controller",
//    "dynamixel",
//    "manipulation",
//};



void miniros_msg_init();
void miniros_msg_send(uint8_t *msg_dest, uint8_t msg_id, uint8_t *msg_buf);
void miniros_msg_module_register(uint8_t *module_name,void (*handler)(uint8_t msg_id,uint8_t *msg_buf));

#endif /* __MESSAGE_H */






