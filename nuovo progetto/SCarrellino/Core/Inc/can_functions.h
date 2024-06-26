#include "main.h"
#define v_cell_id           3u
#define tlb_battery_shut_id 4u


void can_send_msg(uint32_t id);
HAL_StatusTypeDef can_send(CAN_HandleTypeDef *hcan, uint8_t *buffer, CAN_TxHeaderTypeDef *header);
HAL_StatusTypeDef can_wait(CAN_HandleTypeDef *hcan, uint8_t timeout);

void can_rx_routine(void);
void can_tx_1();
void can_tx_2();

