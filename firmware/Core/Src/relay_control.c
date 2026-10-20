#include "relay_control.h"
#include <stdio.h>

/*
 * Failsafe: a lost browser/serial connection must not leave the relay
 * energized indefinitely.  A normal press/release is much shorter.
 */
/* PB0 drives the relay/MOSFET stage that powers the water-bead motor.
 * Keep this timeout active even when the browser or SBC disappears. */
#define RELAY_MAX_ON_TIME_MS 3000U
#define RELAY_ACTIVE_HIGH 1U

static uint8_t relay_on;
static uint32_t relay_on_tick;

static void Relay_Write(uint8_t on)
{
#if RELAY_ACTIVE_HIGH
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

void Relay_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Set the latch low before changing PB0 to an output. */
    Relay_Write(0U);

    gpio_init.Pin = RELAY_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_GPIO_PORT, &gpio_init);

    relay_on = 0U;
    relay_on_tick = 0U;
    Relay_Off();
}

void Relay_On(void)
{
    Relay_Write(1U);
    relay_on = 1U;
    relay_on_tick = HAL_GetTick();

    /*
     * Explicit acknowledgement for the host UI: this is emitted only
     * after PB0 has been driven high, so it verifies the STM32-side
     * command path independently of the relay hardware.
     */
    printf("[RELAY] fire_ack pb0=%u\r\n",
           (unsigned int)(relay_on ? 1U : 0U));
}

void Relay_Off(void)
{
    uint8_t was_on = relay_on;

    Relay_Write(0U);
    relay_on = 0U;
    if (was_on != 0U)
    {
        printf("[RELAY] off_ack pb0=0\r\n");
    }
}

void Relay_Poll(void)
{
    if (relay_on != 0U &&
        (HAL_GetTick() - relay_on_tick) >= RELAY_MAX_ON_TIME_MS)
    {
        Relay_Off();
    }
}

uint8_t Relay_IsOn(void)
{
    return relay_on;
}
