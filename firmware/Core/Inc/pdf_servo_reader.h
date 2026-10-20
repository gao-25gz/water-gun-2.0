#ifndef __PDF_SERVO_READER_H
#define __PDF_SERVO_READER_H

#include "main.h"

void PDF_Servo_TransparentBridge(UART_HandleTypeDef *servo_uart,
                                 UART_HandleTypeDef *host_uart);

#endif
