#include "mpu6050.h"
#include "stdio.h"
#include "math.h"

/* I2C1 句柄，仅本文件内使用 */
static I2C_HandleTypeDef hi2c1;

/* MPU6050 寄存器地址 */
#define MPU6050_ADDR          (0x68U << 1)
#define MPU6050_WHO_AM_I      0x75U
#define MPU6050_PWR_MGMT_1    0x6BU
#define MPU6050_CONFIG        0x1AU
#define MPU6050_GYRO_CONFIG   0x1BU
#define MPU6050_ACCEL_CONFIG  0x1CU
#define MPU6050_ACCEL_XOUT_H  0x3BU
#define MPU6050_I2C_TIMEOUT   100U
/* 连续读失败多少次后软复位 MPU6050（5ms 采样 × 50 ≈ 250ms） */
#define MPU6050_MAX_FAIL_COUNT 50U
#define MPU6050_PWR_MGMT_1_DEVICE_RESET 0x80U

/* 量程灵敏度：陀螺 ±250dps、加速度 ±2g */
#define MPU6050_GYRO_LSB      131.0f
#define MPU6050_ACCEL_LSB     16384.0f

/* DLPF_CFG=3：加速度 44Hz / 陀螺 42Hz 硬件低通。清掉陀螺原始噪声后，
 * 补偿闭环的 Kd（作用在 rate_y 上）才敢往上抬而不抖。 */
#define MPU6050_DLPF_CFG      0x03U

/* 互补滤波参数 */
#define MPU6050_FILTER_ALPHA      0.98f
#define MPU6050_SAMPLE_PERIOD_MS  5U
#define MPU6050_RAD_TO_DEG        57.29578f

/* 轴映射符号：实测若方向反了，改这里的正负号即可 */
#define MPU6050_PITCH_SIGN    1.0f
#define MPU6050_ROLL_SIGN     1.0f

static MPU6050_AttitudeData mpu6050_attitude;
static float pitch_filtered;
static float roll_filtered;
static float pitch_zero;
static float roll_zero;
static uint32_t last_sample_tick;
static uint32_t mpu6050_fail_count = 0U;

void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000U;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0U;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    if (hi2c->Instance != I2C1)
    {
        return;
    }

    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6 = I2C1_SCL，PB7 = I2C1_SDA，开漏复用 */
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            data, len, MPU6050_I2C_TIMEOUT) == HAL_OK;
}

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1U, MPU6050_I2C_TIMEOUT) == HAL_OK;
}

/* F1 的 I2C 卡死(BUSY 锁死)时，翻转 SCL 当 GPIO 无法复位内部状态机；
 * HAL_I2C_Init 内部的 SWRST 会完整复位外设并重配时序，是标准恢复做法。 */
static void MPU6050_I2C_RecoverBus(void)
{
    HAL_I2C_Init(&hi2c1);
}

static void MPU6050_SoftReset(void)
{
    /* 先恢复总线，再软复位 MPU6050，尝试让通信自愈 */
    MPU6050_I2C_RecoverBus();

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_DEVICE_RESET);
    HAL_Delay(100U);

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00U);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00U);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00U);
    MPU6050_WriteReg(MPU6050_CONFIG, MPU6050_DLPF_CFG);

    mpu6050_attitude.valid = 0U;
    mpu6050_fail_count = 0U;
    last_sample_tick = HAL_GetTick();
    printf("[MPU6050] soft reset (recover from read failures)\r\n");
}

void MPU6050_Init(void)
{
    uint8_t who_am_i = 0U;

    printf("[MPU6050] init start\r\n");

    mpu6050_attitude.valid = 0U;
    mpu6050_attitude.frame_count = 0U;
    pitch_filtered = 0.0f;
    roll_filtered = 0.0f;
    pitch_zero = 0.0f;
    roll_zero = 0.0f;
    last_sample_tick = HAL_GetTick();

    MPU6050_I2C_RecoverBus();

    if (!MPU6050_ReadReg(MPU6050_WHO_AM_I, &who_am_i, 1U) || who_am_i != 0x68U)
    {
        printf("[MPU6050] init failed (who_am_i=0x%02X)\r\n", who_am_i);
        return;
    }

    /* 唤醒；陀螺 ±250dps；加速度 ±2g */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00U);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00U);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00U);
    MPU6050_WriteReg(MPU6050_CONFIG, MPU6050_DLPF_CFG);

    printf("[MPU6050] ready, PB6/PB7, 100kHz\r\n");
}

static void MPU6050_UpdateFilter(int16_t ax, int16_t ay, int16_t az,
                                 int16_t gx, int16_t gy, int16_t gz)
{
    float accel_x = (float)ax / MPU6050_ACCEL_LSB;
    float accel_y = (float)ay / MPU6050_ACCEL_LSB;
    float accel_z = (float)az / MPU6050_ACCEL_LSB;
    float gyro_x = (float)gx / MPU6050_GYRO_LSB;
    float gyro_y = (float)gy / MPU6050_GYRO_LSB;
    float gyro_z = (float)gz / MPU6050_GYRO_LSB;

    /* 由重力方向计算静态倾角 */
    float accel_pitch = atan2f(accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * MPU6050_RAD_TO_DEG;
    float accel_roll  = atan2f(accel_y, sqrtf(accel_x * accel_x + accel_z * accel_z)) * MPU6050_RAD_TO_DEG;

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_sample_tick) / 1000.0f;
    last_sample_tick = now;
    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = (float)MPU6050_SAMPLE_PERIOD_MS / 1000.0f;
    }

    /* 俯仰绕 Y 轴（用 gyro_y），横滚绕 X 轴（用 gyro_x），符号用宏调整 */
    float pitch_gyro = gyro_y * MPU6050_PITCH_SIGN;
    float roll_gyro  = gyro_x * MPU6050_ROLL_SIGN;

    pitch_filtered = MPU6050_FILTER_ALPHA * (pitch_filtered + pitch_gyro * dt)
                     + (1.0f - MPU6050_FILTER_ALPHA) * accel_pitch;
    roll_filtered  = MPU6050_FILTER_ALPHA * (roll_filtered + roll_gyro * dt)
                     + (1.0f - MPU6050_FILTER_ALPHA) * accel_roll;

    mpu6050_attitude.rate_x = gyro_x;
    mpu6050_attitude.rate_y = gyro_y;
    mpu6050_attitude.rate_z = gyro_z;
    mpu6050_attitude.gyro_x = gyro_x;
    mpu6050_attitude.gyro_y = gyro_y;
    mpu6050_attitude.gyro_z = gyro_z;

    mpu6050_attitude.raw_pitch = pitch_filtered;
    mpu6050_attitude.raw_roll  = roll_filtered;
    mpu6050_attitude.raw_yaw   = 0.0f;

    mpu6050_attitude.pitch = pitch_filtered - pitch_zero;
    mpu6050_attitude.roll  = roll_filtered - roll_zero;
    mpu6050_attitude.yaw   = 0.0f;

    mpu6050_attitude.timestamp_ms = now;
    mpu6050_attitude.frame_count++;
    mpu6050_attitude.valid = 1U;
}

void MPU6050_Poll(void)
{
    uint8_t buf[14];
    int16_t ax, ay, az, gx, gy, gz;

    if ((HAL_GetTick() - last_sample_tick) < MPU6050_SAMPLE_PERIOD_MS)
    {
        return;
    }

    /* 运行中总线卡死时先就地恢复，放弃本次采样，避免阻塞主循环 */
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
    {
        MPU6050_I2C_RecoverBus();
        last_sample_tick = HAL_GetTick();
        return;
    }

    /* 从 ACCEL_XOUT_H 连续读 14 字节：accel(6) + temp(2) + gyro(6) */
    if (!MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H, buf, 14U))
    {
        /* 读失败：先标记数据无效，再计数；连续失败达到阈值后软复位自愈 */
        mpu6050_attitude.valid = 0U;
        mpu6050_fail_count++;
        if (mpu6050_fail_count >= MPU6050_MAX_FAIL_COUNT)
        {
            MPU6050_SoftReset();
        }
        else
        {
            MPU6050_I2C_RecoverBus();
        }
        return;
    }

    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    gx = (int16_t)((buf[8] << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);

    mpu6050_fail_count = 0U;
    MPU6050_UpdateFilter(ax, ay, az, gx, gy, gz);
}

uint8_t MPU6050_GetAttitude(MPU6050_AttitudeData *data)
{
    if (data == 0)
    {
        return 0U;
    }

    __disable_irq();
    *data = mpu6050_attitude;
    __enable_irq();
    return data->valid;
}

void MPU6050_CalibrateZero(void)
{
    if (mpu6050_attitude.valid != 0U)
    {
        pitch_zero = pitch_filtered;
        roll_zero = roll_filtered;
    }
}
