#include "gimbal_control.h"
#include "fashion_star_uart_servo.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "user_uart.h"
#include "mpu6050.h"
#include "relay_control.h"
#include "ring_buffer.h"

#define GIMBAL_YAW_SERVO_ID 2
#define GIMBAL_PITCH_SERVO_ID 1

#define GIMBAL_YAW_INIT_ANGLE 0.0f
#define GIMBAL_PITCH_INIT_ANGLE 0.0f

#define GIMBAL_YAW_MIN_ANGLE -90.0f
#define GIMBAL_YAW_MAX_ANGLE 90.0f
#define GIMBAL_PITCH_CONTROL_MIN_ANGLE -45.0f
#define GIMBAL_PITCH_CONTROL_MAX_ANGLE 40.0f
/* Physical pitch-servo limits. */
#define GIMBAL_PITCH_SERVO_MIN_ANGLE -50.0f
#define GIMBAL_PITCH_SERVO_MAX_ANGLE 40.0f
/* Logical pitch remains the user-facing target; output mapping is below. */
#define GIMBAL_PITCH_OUTPUT_MIN_ANGLE GIMBAL_PITCH_SERVO_MIN_ANGLE
#define GIMBAL_PITCH_OUTPUT_MAX_ANGLE GIMBAL_PITCH_SERVO_MAX_ANGLE
/* Manual logical pitch follows the ID1 servo's physical direction. */
#define GIMBAL_PITCH_OUTPUT_SIGN (-1.0f)
/* 物理零点标定：补偿安装/装配造成的整框旋转，对所有指令生效（非仅回零）。
 * 正方向 = R（右）/ U（上）；若实测方向反了，把对应符号取反。 */
#define GIMBAL_YAW_ZERO_OFFSET 12.0f
#define GIMBAL_PITCH_ZERO_OFFSET 10.0f
#define GIMBAL_PITCH_COMPENSATION_KP 1.20f 
/* Rate damping gain; raise if compensation oscillates. */
/* 调参临时值：置 0 做纯 P 找 Kp 临界点，找到后按 0.7~0.8× 回退再调此值。 */
#define GIMBAL_PITCH_COMPENSATION_KD 0.01f
/* Damping must share the feedback sign: accel angle and gyro_y flip together. */
#define GIMBAL_PITCH_DAMPING_SIGN (-1.0f)
/* Dead zone only rejects filter noise/drift.  A large zone zeroed the P-term
 * for small tilts, leaving Kd*gyro-noise alone to drive the servo (jitter). */
#define GIMBAL_PITCH_COMPENSATION_START_ERROR 2.0f
/* Measured on base-mounted MPU: raw pitch is opposite to barrel elevation. */
#define GIMBAL_PITCH_FEEDBACK_SIGN (-1.0f)
#define GIMBAL_COMPENSATION_PERIOD_MS 20U
#define GIMBAL_COMPENSATION_MIN_OUTPUT_DELTA 0.50f
/* 手动模式下，停手后多久自动切回补偿模式。需覆盖舵机走完(约50ms)+互补滤波
 * 收敛(约250ms)；太短则切回时 feedback 还没跟上 target，会补一下再回。 */
#define GIMBAL_MANUAL_RESUME_MS 1000U
#define GIMBAL_MANUAL_SERVO_SEND_PERIOD_MS 20U
#define GIMBAL_STATUS_PRINT_PERIOD_MS 500U
#define GIMBAL_PITCH_TELEMETRY_PERIOD_MS 200U
#define GIMBAL_PITCH_WAIT_PRINT_PERIOD_MS 1000U

#define GIMBAL_STEP_ANGLE 1.0f
#define GIMBAL_MOVE_TIME_MS 50U
#define GIMBAL_INIT_TIME_MS 1500U
#define GIMBAL_STARTUP_SETTLE_MS 300U
#define GIMBAL_INIT_RETRY_DELAY_MS 200U
#define GIMBAL_MOVE_SPEED_DPS 450.0f
#define GIMBAL_ACCEL_TIME_MS 20U
#define GIMBAL_DECEL_TIME_MS 20U
#define GIMBAL_SERVO_POWER_MW 2000U

#define GIMBAL_CMD_BUF_SIZE 48
#define GIMBAL_RX_BUF_SIZE 128

static uint8_t gimbal_rx_buf[GIMBAL_RX_BUF_SIZE + 1];
static RingBufferTypeDef gimbal_rx_ring;
static uint8_t gimbal_rx_byte;
static char gimbal_cmd_buf[GIMBAL_CMD_BUF_SIZE];
static volatile uint8_t gimbal_cmd_ready = 0;
static volatile uint8_t gimbal_cmd_len = 0;
static uint8_t gimbal_cmd_index = 0;

static float gimbal_yaw_angle = GIMBAL_YAW_INIT_ANGLE;
static float gimbal_pitch_angle = GIMBAL_PITCH_INIT_ANGLE;
/* Reserved for the attitude-compensation loop; zero keeps manual control unchanged. */
static float gimbal_pitch_compensation = 0.0f;
/* Operating mode: 1 = compensation (level-hold), 0 = manual.
 * Any manual command drops to manual mode; it auto-resumes after
 * GIMBAL_MANUAL_RESUME_MS of no commands, or 'Z' resumes it immediately. */
static uint8_t gimbal_pitch_compensation_enabled = 1U;
static uint8_t gimbal_pitch_compensation_zeroed = 0U;
static uint32_t gimbal_last_attitude_frame = 0U;
static uint32_t gimbal_last_compensation_tick = 0U;
static uint32_t gimbal_last_manual_command_tick = 0U;
static uint32_t gimbal_last_status_print_tick = 0U;
static uint32_t gimbal_last_pitch_telemetry_tick = 0U;
static uint32_t gimbal_last_pitch_wait_print_tick = 0U;
static uint32_t gimbal_last_manual_servo_send_tick = 0U;
static uint8_t gimbal_manual_output_dirty = 0U;
static float gimbal_last_yaw_servo_angle = 0.0f;
static float gimbal_last_pitch_servo_angle = 0.0f;

static void Gimbal_SendServoAngle(uint8_t servo_id, float angle,
                                  uint32_t interval_ms)
{
    if (interval_ms == GIMBAL_MOVE_TIME_MS)
    {
        FSUS_SetServoAngleMTurnByVelocity(
            &FSUS_usart1, servo_id, angle, GIMBAL_MOVE_SPEED_DPS,
            GIMBAL_ACCEL_TIME_MS, GIMBAL_DECEL_TIME_MS,
            GIMBAL_SERVO_POWER_MW);
    }
    else
    {
        FSUS_SetServoAngleMTurn(
            &FSUS_usart1, servo_id, angle, interval_ms,
            GIMBAL_SERVO_POWER_MW);
    }
}

static void Gimbal_HandleRxByte(uint8_t rx)
{
    if (gimbal_cmd_ready != 0)
    {
        return;
    }

    /*
     * USART2 is an ASCII command channel.  Ignore binary noise instead of
     * buffering it and printing it back as an "unknown command".  This also
     * prevents a miswired STorM32/servo TX line from flooding the console.
     */
    if ((rx < 0x20U && rx != '\r' && rx != '\n') || rx > 0x7EU)
    {
        gimbal_cmd_index = 0U;
        return;
    }

    if (rx >= 'a' && rx <= 'z')
    {
        rx = (uint8_t)(rx - 'a' + 'A');
    }

    if (gimbal_cmd_index == 0U &&
        (rx == 'L' || rx == 'R' || rx == 'U' ||
        rx == 'D' || rx == 'C' || rx == 'S' ||
        rx == 'Z'))
    {
        gimbal_cmd_buf[0] = (char)rx;
        gimbal_cmd_len = 1;
        gimbal_cmd_ready = 1;
        gimbal_cmd_index = 0;
    }
    else if (rx == '\n' || rx == '\r')
    {
        if (gimbal_cmd_index > 0)
        {
            gimbal_cmd_len = gimbal_cmd_index;
            gimbal_cmd_ready = 1;
        }
        gimbal_cmd_index = 0;
    }
    else if (gimbal_cmd_index < (GIMBAL_CMD_BUF_SIZE - 1))
    {
        gimbal_cmd_buf[gimbal_cmd_index] = (char)rx;
        gimbal_cmd_index++;

        /*
         * FIRE/OFF are also accepted without a trailing CR/LF.  This is
         * useful with serial assistants whose "send ASCII" mode transmits
         * exactly the typed characters and no line ending.
         */
        if ((gimbal_cmd_index == 4U) &&
            memcmp(gimbal_cmd_buf, "FIRE", 4U) == 0)
        {
            gimbal_cmd_len = 4U;
            gimbal_cmd_ready = 1U;
            gimbal_cmd_index = 0U;
        }
        else if ((gimbal_cmd_index == 3U) &&
                 memcmp(gimbal_cmd_buf, "OFF", 3U) == 0)
        {
            gimbal_cmd_len = 3U;
            gimbal_cmd_ready = 1U;
            gimbal_cmd_index = 0U;
        }
    }
    else
    {
        gimbal_cmd_index = 0;
    }
}

static float Gimbal_ParseAngle(const char *text)
{
    float value = 0.0f;
    float decimal = 0.1f;
    float sign = 1.0f;
    uint8_t after_point = 0;

    if (*text == '-')
    {
        sign = -1.0f;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    while (*text != '\0')
    {
        if (*text == '.')
        {
            after_point = 1;
        }
        else if (*text >= '0' && *text <= '9')
        {
            if (after_point == 0)
            {
                value = value * 10.0f + (float)(*text - '0');
            }
            else
            {
                value += decimal * (float)(*text - '0');
                decimal *= 0.1f;
            }
        }
        else
        {
            break;
        }

        text++;
    }

    return sign * value;
}

static float Gimbal_Limit(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static float Gimbal_GetYawServoAngle(float logical_yaw)
{
    return logical_yaw + GIMBAL_YAW_ZERO_OFFSET;
}

static float Gimbal_GetPitchServoAngle(float logical_pitch)
{
    float output_pitch = GIMBAL_PITCH_OUTPUT_SIGN *
                         (logical_pitch + GIMBAL_PITCH_ZERO_OFFSET +
                          gimbal_pitch_compensation);

    output_pitch = Gimbal_Limit(
        output_pitch,
        GIMBAL_PITCH_OUTPUT_MIN_ANGLE,
        GIMBAL_PITCH_OUTPUT_MAX_ANGLE);

    return Gimbal_Limit(
        output_pitch,
        GIMBAL_PITCH_SERVO_MIN_ANGLE,
        GIMBAL_PITCH_SERVO_MAX_ANGLE);
}

static uint8_t Gimbal_AngleChanged(float a, float b)
{
    float delta = a - b;

    return (delta > 0.001f || delta < -0.001f);
}

static void Gimbal_SendCurrentOutput(uint32_t interval_ms, uint8_t force_both)
{
    float new_yaw_servo_angle;
    float new_pitch_servo_angle;

    new_yaw_servo_angle = Gimbal_GetYawServoAngle(gimbal_yaw_angle);
    new_pitch_servo_angle = Gimbal_GetPitchServoAngle(gimbal_pitch_angle);

    /*
     * During interactive control, use velocity mode.  Sending a new
     * time-based trajectory every few milliseconds can repeatedly replace
     * the previous trajectory before the servo has started moving.
     * Initialization/centering/stop keep the original interval mode.
     */
    if (force_both ||
        Gimbal_AngleChanged(new_yaw_servo_angle, gimbal_last_yaw_servo_angle))
    {
        Gimbal_SendServoAngle(GIMBAL_YAW_SERVO_ID,
                              new_yaw_servo_angle, interval_ms);
        gimbal_last_yaw_servo_angle = new_yaw_servo_angle;
    }

    if (force_both ||
        Gimbal_AngleChanged(new_pitch_servo_angle,
                            gimbal_last_pitch_servo_angle))
    {
        Gimbal_SendServoAngle(GIMBAL_PITCH_SERVO_ID,
                              new_pitch_servo_angle,
                              interval_ms);
        gimbal_last_pitch_servo_angle = new_pitch_servo_angle;
    }
}

static void Gimbal_FlushManualOutput(void)
{
    uint32_t now = HAL_GetTick();

    if (gimbal_manual_output_dirty == 0U)
    {
        return;
    }

    if ((now - gimbal_last_manual_servo_send_tick) <
        GIMBAL_MANUAL_SERVO_SEND_PERIOD_MS)
    {
        return;
    }

    Gimbal_SendCurrentOutput(GIMBAL_MOVE_TIME_MS, 0U);
    gimbal_manual_output_dirty = 0U;
    gimbal_last_manual_servo_send_tick = now;
}

static void Gimbal_SetAngle(float yaw, float pitch, uint32_t interval_ms)
{
    uint32_t now = HAL_GetTick();
    uint8_t interactive = (interval_ms == GIMBAL_MOVE_TIME_MS);

    /*
     * Any manual command switches to manual mode: compensation is suspended,
     * but its accumulated correction is FROZEN rather than cleared.  Clearing
     * it dropped the barrel by the standing correction and snapped it back
     * when compensation resumed; freezing keeps mode entry/exit bumpless.
     * The timer restarts on every command, so compensation only resumes once
     * the operator stops.
     */
    gimbal_pitch_compensation_enabled = 0U;
    gimbal_last_manual_command_tick = now;

    gimbal_yaw_angle = Gimbal_Limit(yaw, GIMBAL_YAW_MIN_ANGLE, GIMBAL_YAW_MAX_ANGLE);
    gimbal_pitch_angle = Gimbal_Limit(
        pitch,
        GIMBAL_PITCH_CONTROL_MIN_ANGLE,
        GIMBAL_PITCH_CONTROL_MAX_ANGLE);

    if (interactive != 0U)
    {
        gimbal_manual_output_dirty = 1U;
        Gimbal_FlushManualOutput();
    }
    else
    {
        Gimbal_SendCurrentOutput(interval_ms, 1U);
        gimbal_manual_output_dirty = 0U;
        gimbal_last_manual_servo_send_tick = now;
    }

    if ((now - gimbal_last_status_print_tick) >=
        GIMBAL_STATUS_PRINT_PERIOD_MS)
    {
        printf("[GIMBAL] yaw=%.1f pitch=%.1f\r\n",
               gimbal_yaw_angle, gimbal_pitch_angle);
        gimbal_last_status_print_tick = now;
    }
}

static void Gimbal_UpdatePitchCompensation(void)
{
    MPU6050_AttitudeData attitude;
    float feedback_pitch;
    float error;
    float new_compensation;
    float new_servo_angle;
    float compensation_min;
    float compensation_max;
    float pitch_rate;
    uint32_t now = HAL_GetTick();

    if ((now - gimbal_last_compensation_tick) <
        GIMBAL_COMPENSATION_PERIOD_MS)
    {
        return;
    }

    /* Manual mode: stay suspended until the operator stops, then resume.
     * The target already follows the last command, so resuming holds that
     * aim against base motion (it does not snap back to zero). */
    if (gimbal_pitch_compensation_enabled == 0U)
    {
        if ((now - gimbal_last_manual_command_tick) <
            GIMBAL_MANUAL_RESUME_MS)
        {
            return;
        }

        gimbal_pitch_compensation_enabled = 1U;
        /* Drop attitude frames captured while the operator was moving. */
        if (MPU6050_GetAttitude(&attitude) != 0U)
        {
            gimbal_last_attitude_frame = attitude.frame_count;
        }
        printf("[GIMBAL] manual ended, compensation resumed\r\n");
        return;
    }

    if (MPU6050_GetAttitude(&attitude) == 0U ||
        attitude.frame_count == gimbal_last_attitude_frame)
    {
        return;
    }

    gimbal_last_compensation_tick = now;
    gimbal_last_attitude_frame = attitude.frame_count;

    /* Establish a safe startup reference from the first valid frame. */
    if (gimbal_pitch_compensation_zeroed == 0U)
    {
        MPU6050_CalibrateZero();
        gimbal_pitch_compensation_zeroed = 1U;
        gimbal_pitch_compensation = 0.0f;
        printf("[GIMBAL] startup compensation zeroed\r\n");
        return;
    }

    feedback_pitch = GIMBAL_PITCH_FEEDBACK_SIGN * attitude.pitch;
    /*
     * MPU6050 is mounted on the BASE, so feedback_pitch is the base tilt, not
     * the barrel angle.  The loop only cancels base motion; the manual target
     * is applied purely through the servo output below.  Subtracting the target
     * here (error = gimbal_pitch_angle - feedback_pitch) made comp scale with
     * the manual angle (comp = Kp*L while the base is still), so resuming
     * compensation after a manual move stepped the output by Kp*L.
     */
    error = -feedback_pitch;

    if (error < GIMBAL_PITCH_COMPENSATION_START_ERROR &&
        error > -GIMBAL_PITCH_COMPENSATION_START_ERROR)
    {
        error = 0.0f;
    }

    pitch_rate = GIMBAL_PITCH_DAMPING_SIGN * attitude.rate_y;
    new_compensation = GIMBAL_PITCH_COMPENSATION_KP * error
                       - GIMBAL_PITCH_COMPENSATION_KD * pitch_rate;

    /* Keep target + correction inside the real mechanical limits. */
    if (GIMBAL_PITCH_OUTPUT_SIGN > 0.0f)
    {
        compensation_min = GIMBAL_PITCH_SERVO_MIN_ANGLE - gimbal_pitch_angle;
        compensation_max = GIMBAL_PITCH_SERVO_MAX_ANGLE - gimbal_pitch_angle;
    }
    else
    {
        compensation_min = -GIMBAL_PITCH_SERVO_MAX_ANGLE - gimbal_pitch_angle;
        compensation_max = -GIMBAL_PITCH_SERVO_MIN_ANGLE - gimbal_pitch_angle;
    }
    gimbal_pitch_compensation = Gimbal_Limit(
        new_compensation,
        compensation_min,
        compensation_max);

    new_servo_angle = Gimbal_GetPitchServoAngle(gimbal_pitch_angle);
    if ((new_servo_angle - gimbal_last_pitch_servo_angle) >
            GIMBAL_COMPENSATION_MIN_OUTPUT_DELTA ||
        (gimbal_last_pitch_servo_angle - new_servo_angle) >
            GIMBAL_COMPENSATION_MIN_OUTPUT_DELTA)
    {
        Gimbal_SendServoAngle(
            GIMBAL_PITCH_SERVO_ID, new_servo_angle,
            GIMBAL_MOVE_TIME_MS);
        gimbal_last_pitch_servo_angle = new_servo_angle;
    }
}

/*
 * Only the pitch value needed by the browser UI is forwarded on USART2.
 * The full STorM32 gyro/attitude stream remains internal to the STM32.
 */
static void Gimbal_PrintPitchTelemetry(void)
{
    MPU6050_AttitudeData attitude;
    uint32_t now = HAL_GetTick();

    if ((now - gimbal_last_pitch_telemetry_tick) <
        GIMBAL_PITCH_TELEMETRY_PERIOD_MS)
    {
        return;
    }

    if (MPU6050_GetAttitude(&attitude) == 0U)
    {
        if ((now - gimbal_last_pitch_wait_print_tick) >=
            GIMBAL_PITCH_WAIT_PRINT_PERIOD_MS)
        {
            printf("[PITCH] waiting mpu6050 data\r\n");
            gimbal_last_pitch_wait_print_tick = now;
        }
        return;
    }

    printf("[PITCH] raw=%.2f relative=%.2f comp=%.2f enabled=%u frame=%lu\r\n",
           attitude.raw_pitch,
           attitude.pitch,
           gimbal_pitch_compensation,
           (unsigned int)gimbal_pitch_compensation_enabled,
           (unsigned long)attitude.frame_count);
    gimbal_last_pitch_telemetry_tick = now;
}

static void Gimbal_ProcessCommand(char *cmd)
{
    char *comma;
    float yaw;
    float pitch;
    MPU6050_AttitudeData attitude;

    if (strcmp(cmd, "L") == 0)
    {
        Gimbal_SetAngle(gimbal_yaw_angle - GIMBAL_STEP_ANGLE,
                        gimbal_pitch_angle, GIMBAL_MOVE_TIME_MS);
    }
    else if (strcmp(cmd, "R") == 0)
    {
        Gimbal_SetAngle(gimbal_yaw_angle + GIMBAL_STEP_ANGLE,
                        gimbal_pitch_angle, GIMBAL_MOVE_TIME_MS);
    }
    else if (strcmp(cmd, "U") == 0)
    {
        Gimbal_SetAngle(gimbal_yaw_angle,
                        gimbal_pitch_angle + GIMBAL_STEP_ANGLE,
                        GIMBAL_MOVE_TIME_MS);
    }
    else if (strcmp(cmd, "D") == 0)
    {
        Gimbal_SetAngle(gimbal_yaw_angle,
                        gimbal_pitch_angle - GIMBAL_STEP_ANGLE,
                        GIMBAL_MOVE_TIME_MS);
    }
    else if (strcmp(cmd, "C") == 0)
    {
        Gimbal_SetAngle(GIMBAL_YAW_INIT_ANGLE,
                        GIMBAL_PITCH_INIT_ANGLE,
                        GIMBAL_INIT_TIME_MS);
    }
    else if (cmd[0] == 'Y' && cmd[1] == ':')
    {
        yaw = Gimbal_ParseAngle(&cmd[2]);
        Gimbal_SetAngle(yaw, gimbal_pitch_angle, GIMBAL_MOVE_TIME_MS);
    }
    else if (cmd[0] == 'P' && cmd[1] == ':')
    {
        pitch = Gimbal_ParseAngle(&cmd[2]);
        Gimbal_SetAngle(gimbal_yaw_angle, pitch, GIMBAL_MOVE_TIME_MS);
    }
    else if (cmd[0] == 'A' && cmd[1] == ':')
    {
        comma = strchr(&cmd[2], ',');
        if (comma != NULL)
        {
            *comma = '\0';
            yaw = Gimbal_ParseAngle(&cmd[2]);
            pitch = Gimbal_ParseAngle(comma + 1);
            Gimbal_SetAngle(yaw, pitch, GIMBAL_MOVE_TIME_MS);
        }
        else
        {
            printf("[GIMBAL] bad command: %s\r\n", cmd);
        }
    }
    else if (strcmp(cmd, "S") == 0)
    {
        Gimbal_SetAngle(gimbal_yaw_angle, gimbal_pitch_angle, 0U);
    }
    else if (strcmp(cmd, "Z") == 0)
    {
        if (MPU6050_GetAttitude(&attitude) != 0U)
        {
            MPU6050_CalibrateZero();
            gimbal_pitch_compensation = 0.0f;
            gimbal_last_attitude_frame = attitude.frame_count;
            gimbal_pitch_compensation_zeroed = 1U;
            gimbal_pitch_compensation_enabled = 1U;
            printf("[GIMBAL] pitch compensation enabled\r\n");
        }
        else
        {
            printf("[GIMBAL] cannot zero: MPU6050 data not ready\r\n");
        }
    }
    else if (strcmp(cmd, "FIRE") == 0 || strcmp(cmd, "ON") == 0)
    {
        Relay_On();
        printf("[RELAY] on\r\n");
    }
    else if (strcmp(cmd, "OFF") == 0)
    {
        Relay_Off();
        printf("[RELAY] off\r\n");
    }
    else
    {
        printf("[GIMBAL] unknown command: %s\r\n", cmd);
    }
}

void Gimbal_Init(void)
{
    printf("[GIMBAL] init\r\n");

    gimbal_yaw_angle = GIMBAL_YAW_INIT_ANGLE;
    gimbal_pitch_angle = GIMBAL_PITCH_INIT_ANGLE;
    gimbal_pitch_compensation = 0.0f;
    gimbal_pitch_compensation_enabled = 1U;
    gimbal_pitch_compensation_zeroed = 0U;
    gimbal_last_attitude_frame = 0U;
    gimbal_last_compensation_tick = HAL_GetTick();
    gimbal_last_manual_command_tick = HAL_GetTick();
    gimbal_last_status_print_tick = HAL_GetTick();
    gimbal_last_pitch_telemetry_tick = HAL_GetTick();
    gimbal_last_pitch_wait_print_tick = HAL_GetTick();
    gimbal_last_manual_servo_send_tick = HAL_GetTick();
    gimbal_manual_output_dirty = 0U;
    gimbal_last_yaw_servo_angle = 0.0f;
    gimbal_last_pitch_servo_angle = 0.0f;

    /*
     * Give the external servo power rail time to start before the
     * first bus frame.  Then resend the startup target once: this covers
     * the common case where the servo is still booting when the MCU sends
     * the first packet.
     */
    HAL_Delay(GIMBAL_STARTUP_SETTLE_MS);
    Gimbal_SetAngle(gimbal_yaw_angle, gimbal_pitch_angle, GIMBAL_INIT_TIME_MS);
    HAL_Delay(GIMBAL_INIT_TIME_MS + 200U);
    Gimbal_SendCurrentOutput(GIMBAL_INIT_TIME_MS, 1U);
    HAL_Delay(GIMBAL_INIT_RETRY_DELAY_MS);
    printf("[GIMBAL] startup position resent\r\n");

    RingBuffer_Init(&gimbal_rx_ring, GIMBAL_RX_BUF_SIZE, gimbal_rx_buf);
    HAL_UART_Receive_IT(&huart2, &gimbal_rx_byte, 1U);

    /* Startup position was sent via Gimbal_SetAngle (which drops to manual
     * mode); begin in compensation mode so level-hold engages immediately. */
    gimbal_pitch_compensation_enabled = 1U;

    printf("[GIMBAL] ready\r\n");
    printf("[GIMBAL] rx_interrupt\r\n");
    printf("[GIMBAL] pitch compensation enabled at startup\r\n");
}

void Gimbal_Poll(void)
{
    char cmd[GIMBAL_CMD_BUF_SIZE];
    uint8_t len;
    uint8_t rx;

    while (RingBuffer_GetByteUsed(&gimbal_rx_ring) > 0U)
    {
        rx = RingBuffer_Pop(&gimbal_rx_ring);
        Gimbal_HandleRxByte(rx);
    }

    if (gimbal_cmd_ready == 0)
    {
        Gimbal_FlushManualOutput();
        Gimbal_UpdatePitchCompensation();
        Gimbal_PrintPitchTelemetry();
        return;
    }

    __disable_irq();
    len = gimbal_cmd_len;
    if (len >= GIMBAL_CMD_BUF_SIZE)
    {
        len = GIMBAL_CMD_BUF_SIZE - 1;
    }
    memcpy(cmd, gimbal_cmd_buf, len);
    cmd[len] = '\0';
    gimbal_cmd_ready = 0;
    __enable_irq();

    Gimbal_ProcessCommand(cmd);
    Gimbal_FlushManualOutput();
    Gimbal_UpdatePitchCompensation();
    Gimbal_PrintPitchTelemetry();
}

void Gimbal_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    RingBuffer_Push(&gimbal_rx_ring, gimbal_rx_byte);
    HAL_UART_Receive_IT(&huart2, &gimbal_rx_byte, 1U);
}

void Gimbal_RestartRx(void)
{
    /* 清错误标志（溢出/帧/噪声），否则重启接收后错误中断会立即再触发、卡死主循环 */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    HAL_UART_Receive_IT(&huart2, &gimbal_rx_byte, 1U);
}
