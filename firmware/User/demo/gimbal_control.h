#ifndef __GIMBAL_CONTROL_H__
#define __GIMBAL_CONTROL_H__

#include "main.h"

void Gimbal_Init(void);
void Gimbal_Poll(void);
void Gimbal_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Gimbal_RestartRx(void);

#endif
