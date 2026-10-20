#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核桃派到 STM32 的串口、云台、发射和摄像头现场测试程序。

默认只测试串口和云台，不会触发发射。继电器测试必须同时指定
--test-relay 和 --allow-fire，并且测试前应拆下电机/水弹。
"""

import argparse
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    serial = None


def 输出(名称, 状态, 说明=""):
    """统一打印测试结果。"""
    标记 = "通过" if 状态 else "失败"
    print(f"[{标记}] {名称}{(': ' + 说明) if 说明 else ''}")


def 查找串口():
    """列出核桃派上常见的 UART、USB 串口设备。"""
    设备 = []
    设备.extend(glob.glob("/dev/serial/by-id/*"))
    设备.extend(glob.glob("/dev/ttyS*"))
    设备.extend(glob.glob("/dev/ttyUSB*"))
    设备.extend(glob.glob("/dev/ttyACM*"))
    return list(dict.fromkeys(设备))


def 接收文本(串口对象, 秒数):
    """在指定时间内收集 STM32 返回的 ASCII 行。"""
    行列表 = []
    截止时间 = time.monotonic() + 秒数
    while time.monotonic() < 截止时间:
        数据 = 串口对象.readline()
        if 数据:
            文本 = 数据.decode("ascii", errors="replace").strip()
            if 文本:
                行列表.append(文本)
                print(f"  << {文本}")
    return 行列表


def 发送命令(串口对象, 命令, 等待秒数=1.0):
    """向 USART2 发送一条以换行结束的 ASCII 命令。"""
    print(f"  >> {命令}")
    串口对象.write((命令 + "\n").encode("ascii"))
    串口对象.flush()
    return 接收文本(串口对象, 等待秒数)


def 测试摄像头(设备参数):
    """打开摄像头并读取一帧，验证 V4L2/OpenCV 链路。"""
    try:
        import cv2
    except ImportError:
        输出("摄像头", False, "未安装 python3-opencv")
        return False

    候选 = [设备参数]
    if 设备参数 == "auto":
        候选 = [f"/dev/video{i}" for i in range(8)]

    for 设备 in 候选:
        if not os.path.exists(设备):
            continue
        摄像头 = cv2.VideoCapture(设备, getattr(cv2, "CAP_V4L2", 0))
        try:
            if 摄像头.isOpened():
                成功, 画面 = 摄像头.read()
                if 成功 and 画面 is not None:
                    输出("摄像头", True, f"{设备}，分辨率 {画面.shape[1]}x{画面.shape[0]}")
                    return True
        finally:
            摄像头.release()

    输出("摄像头", False, "没有读取到可用画面")
    return False


def 主程序():
    解析器 = argparse.ArgumentParser(description="核桃派水弹枪项目 2.0 现场测试")
    解析器.add_argument("--port", default="/dev/ttyS2", help="STM32 USART2 设备名")
    解析器.add_argument("--baud", type=int, default=115200, help="串口波特率")
    解析器.add_argument("--camera-device", default="auto", help="摄像头设备名或 auto")
    解析器.add_argument("--skip-camera", action="store_true", help="跳过摄像头测试")
    解析器.add_argument("--test-relay", action="store_true", help="测试 FIRE/OFF")
    解析器.add_argument("--allow-fire", action="store_true", help="确认允许短时触发继电器")
    参数 = 解析器.parse_args()

    print("核桃派水弹枪项目 2.0 测试开始")
    print(f"串口: {参数.port} @ {参数.baud}")
    if serial is None:
        输出("Python 串口模块", False, "请安装 python3-serial")
        return 2

    设备 = 参数.port
    if 设备 == "auto":
        候选 = 查找串口()
        print("发现的串口:", ", ".join(候选) if 候选 else "无")
        输出("串口设备", bool(候选), "请用 --port 指定实际设备" if not 候选 else "")
        return 1 if not 候选 else 2

    if not os.path.exists(设备):
        输出("串口设备", False, f"不存在: {设备}")
        print("提示：在核桃派执行 ls -l /dev/ttyS* /dev/ttyUSB* /dev/ttyACM*")
        return 1

    结果 = []
    串口对象 = None
    try:
        串口对象 = serial.Serial(设备, 参数.baud, timeout=0.2, write_timeout=1)
        串口对象.reset_input_buffer()
        启动日志 = 接收文本(串口对象, 2.0)
        启动正常 = any("[GIMBAL] ready" in 行 for 行 in 启动日志)
        if 启动正常:
            输出("STM32 启动日志", True, "收到 [GIMBAL] ready")
        else:
            print("[提示] 未在打开串口后的缓存中看到 ready，继续用状态日志确认链路")

        回中日志 = 发送命令(串口对象, "C", 1.2)
        回中有回应 = any("[GIMBAL]" in 行 for 行 in 回中日志)
        回中无错误 = not any("unknown command" in 行 or "bad command" in 行 for 行 in 回中日志)
        回中正常 = 回中有回应 and 回中无错误
        输出("云台回中命令 C", 回中正常,
             "收到 STM32 回应" if 回中正常 else "没有收到 STM32 回应，请检查串口设备和固件")
        结果.append(回中正常)
        状态日志 = 启动日志 + 回中日志
        状态正常 = any("[GIMBAL] yaw=" in 行 for 行 in 状态日志)
        输出("STM32 返回状态", 状态正常, "收到云台角度状态" if 状态正常 else "未收到 [GIMBAL] 状态，请检查串口 TX/RX 和固件")
        结果.append(状态正常)

        角度日志 = 发送命令(串口对象, "A:5.0,-5.0", 1.2)
        角度有回应 = any("[GIMBAL]" in 行 for 行 in 角度日志)
        角度无错误 = not any("unknown command" in 行 or "bad command" in 行 for 行 in 角度日志)
        角度正常 = 角度有回应 and 角度无错误
        输出("双轴角度命令 A", 角度正常,
             "收到 STM32 回应，请观察两个舵机动作" if 角度正常
             else "没有收到 STM32 回应，请检查串口设备和固件")
        结果.append(角度正常)

        发送命令(串口对象, "A:0.0,0.0", 1.0)

        if 参数.test_relay:
            if not 参数.allow_fire:
                输出("继电器测试", False, "必须同时指定 --allow-fire，避免误触发")
                结果.append(False)
            else:
                开火日志 = 发送命令(串口对象, "FIRE", 0.8)
                开火正常 = any("fire_ack pb0=1" in 行 for 行 in 开火日志)
                输出("FIRE/PB0", 开火正常, "收到开火回执" if 开火正常 else "未收到 fire_ack")
                结果.append(开火正常)
                关闭日志 = 发送命令(串口对象, "OFF", 0.8)
                关闭正常 = any("off_ack pb0=0" in 行 for 行 in 关闭日志)
                输出("OFF/PB0", 关闭正常, "收到关闭回执" if 关闭正常 else "未收到 off_ack")
                结果.append(关闭正常)
        else:
            print("[跳过] 继电器测试（默认跳过，避免电机误动作）")
    except (serial.SerialException, OSError) as 异常:
        输出("STM32 串口", False, str(异常))
        return 1
    finally:
        if 串口对象 is not None and 串口对象.is_open:
            串口对象.close()

    if not 参数.skip_camera:
        结果.append(测试摄像头(参数.camera_device))
    else:
        print("[跳过] 摄像头测试")

    通过 = all(结果)
    print("测试结束：全部通过" if 通过 else "测试结束：存在失败项目，请按提示排查")
    return 0 if 通过 else 1


if __name__ == "__main__":
    sys.exit(主程序())
