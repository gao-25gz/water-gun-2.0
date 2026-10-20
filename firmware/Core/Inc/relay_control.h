#ifndef __RELAY_CONTROL_H__
#define __RELAY_CONTROL_H__

#include "main.h"

/*
 * RelayTest hardware mapping:
 *   PB0 high -> relay driver on
 *   PB0 low  -> relay driver off
 */
#define RELAY_GPIO_PORT GPIOB
#define RELAY_GPIO_PIN  GPIO_PIN_0

void Relay_Init(void);
void Relay_On(void);
void Relay_Off(void);
void Relay_Poll(void);
uint8_t Relay_IsOn(void);

#endif
