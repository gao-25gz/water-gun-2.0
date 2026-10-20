#include "pdf_servo_reader.h"

#define PDF_SERVO_TIMEOUT_MS 100U
#define PDF_SERVO_MAX_FRAME 260U

static HAL_StatusTypeDef PDF_Servo_ReceiveFrame(
    UART_HandleTypeDef *huart,
    uint8_t *frame,
    uint16_t *frame_length)
{
    uint8_t byte;
    uint8_t length;
    uint16_t payload_length;

    do {
        if (HAL_UART_Receive(huart, &byte, 1U,
                             PDF_SERVO_TIMEOUT_MS) != HAL_OK) {
            return HAL_TIMEOUT;
        }
    } while (byte != 0xFFU);

    frame[0] = 0xFFU;

    if (HAL_UART_Receive(huart, &byte, 1U,
                         PDF_SERVO_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }
    if (byte != 0xFFU) {
        return HAL_ERROR;
    }
    frame[1] = 0xFFU;

    if (HAL_UART_Receive(huart, &frame[2], 2U,
                         PDF_SERVO_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    length = frame[3];
    payload_length = (uint16_t)length + 2U;
    if (payload_length > (PDF_SERVO_MAX_FRAME - 4U)) {
        return HAL_ERROR;
    }

    if (HAL_UART_Receive(huart, &frame[4], length,
                         PDF_SERVO_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    *frame_length = payload_length + 2U;
    return HAL_OK;
}

void PDF_Servo_TransparentBridge(UART_HandleTypeDef *servo_uart,
                                 UART_HandleTypeDef *host_uart)
{
    uint8_t request_frame[PDF_SERVO_MAX_FRAME];
    uint8_t response_frame[PDF_SERVO_MAX_FRAME];
    uint16_t request_length;
    uint16_t response_length;

    while (1) {
        /*
         * Host packet format:
         * FF FF ID Length Instruction Parameter... Checksum
         * Length includes Instruction, Parameters and Checksum.
         */
        if (PDF_Servo_ReceiveFrame(host_uart, request_frame,
                                   &request_length) != HAL_OK) {
            continue;
        }

        if (HAL_HalfDuplex_EnableTransmitter(servo_uart) != HAL_OK) {
            continue;
        }

        if (HAL_UART_Transmit(servo_uart, request_frame, request_length,
                              PDF_SERVO_TIMEOUT_MS) != HAL_OK) {
            continue;
        }

        while (__HAL_UART_GET_FLAG(servo_uart, UART_FLAG_TC) == RESET) {
        }

        if (HAL_HalfDuplex_EnableReceiver(servo_uart) != HAL_OK) {
            continue;
        }

        /*
         * Return the servo status packet unchanged to the host.
         * Broadcast commands normally have no response and time out here.
         */
        if (PDF_Servo_ReceiveFrame(servo_uart, response_frame,
                                   &response_length) == HAL_OK) {
            HAL_UART_Transmit(host_uart, response_frame, response_length,
                              PDF_SERVO_TIMEOUT_MS);
        }
    }
}
