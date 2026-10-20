# 水弹枪火控 / 云台项目 2.0

STM32F103C8T6 主控 + 核桃派 1B 上位机的水弹枪控制系统。仓库只包含两套源码：

- `firmware/`：STM32F103 Keil MDK-ARM 工程（HAL 库）
- `walnutpi/`：核桃派上的 Python 串口 / 摄像头 / 网页控制程序

## 系统链路

```text
USB 摄像头 ──────────────► 核桃派（OpenCV / MJPEG）
电脑浏览器 ── HTTP ──────► 核桃派 server.py
核桃派 USB ── CH340 ─────► STM32 USART2（PA2/PA3，115200 8N1）
STM32 USART1（PA9 半双工）► 总线舵机 ID1=俯仰、ID2=水平
MPU6050 ── I2C1(PB6/PB7) ─► STM32 俯仰姿态补偿
STM32 PB0 ── 光耦/MOSFET/继电器 ──► 发射电机
```

## 硬件连接要点

| 连接 | 说明 |
|---|---|
| 24V 电源 | 板子主电源。首次上电建议限流 0.5~1A；整机联调 3A，最大不超过 5A |
| 核桃派 USB-A ↔ 板子 Type-C | 板子 Type-C 因 `A7↔B7` 飞线只有**一个插入方向**可用；不识别时翻转 180° 再插 |
| USB 线材 | 必须用带数据的 A-to-C 线，不能用纯充电线 |
| 舵机 CN5 | 1=GND，2=+12V，3=PA9（两只舵机 S 并联），4=PA10 不接 |
| 舵机 ID | 必须不同：ID1=俯仰，ID2=水平 |
| MPU6050 CN3 | PB6=SCL，PB7=SDA，3.3V，GND |
| 摄像头 | 直接插核桃派 USB，不经过主板 |
| 发射电机 | 走板上 5V_motor 与继电器/MOSFET，不能从 STM32 3.3V 取电 |

## 编译 STM32 固件

1. 用 Keil MDK-ARM 打开 `firmware/fashionstar-uart-servo-stm32f103-hal-sdk.uvprojx`。
2. 选择目标 `STM32F103C8`，Build。
3. 用 ST-Link（SWD）下载。

注意：USART1 的 PA9 是单线半双工，不要用 CubeMX 重新生成后覆盖掉 `HAL_HalfDuplex_Init`、`GPIO_MODE_AF_OD` 和收发方向切换。

## 启动核桃派上位机

```bash
sudo apt update
sudo apt install -y python3-serial python3-opencv v4l-utils
sudo usermod -aG dialout,video $USER   # 重新登录生效
```

确认设备：

```bash
ls -l /dev/serial/by-id/
v4l2-ctl --list-devices
python3 walnutpi/check_environment.py --serial /dev/ttyUSB0 --camera-device auto
```

启动网页服务：

```bash
python3 walnutpi/gimbal_web/server.py \
  --serial /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baud 115200 \
  --camera-device /dev/v4l/by-id/usb-KINGSEN_KS1A552-video-index0 \
  --camera-width 640 --camera-height 480 --camera-fps 20 \
  --port 8080
```

浏览器访问 `http://<核桃派IP>:8080`。

> 板载 CH340 枚举为 `/dev/ttyUSB0`。如果设备号会漂移，使用上面的 by-id 路径。摄像头也可先用 `--camera-device auto` 自动探测。

## 网页控制

| 操作 | 功能 |
|---|---|
| 按住 W/A/S/D 或方向按钮 | 连续转动云台（1°/20ms） |
| 回零 / C | 两轴回到逻辑 0° |
| 按住“发射”或空格 | 发送 FIRE，PB0 拉高；松开发送 OFF |
| Z | MPU6050 零位校准并开启俯仰补偿 |
| N | 关闭俯仰补偿 |

安全行为：上电 PB0 默认低；发射最长保持 3 秒自动关断；浏览器失焦、松开按钮都会发送 OFF。

## 串口命令

`L` / `R` / `U` / `D` / `C` / `S`、`A:yaw,pitch`、`Z` / `N`、`FIRE` / `OFF`，115200 8N1。

## 说明

- 本仓库不包含 EDA 工程、BOM、合同、保密协议和质量协议。
- 详细操作、上电顺序、故障排查见随项目交付的 PDF 使用说明书。
