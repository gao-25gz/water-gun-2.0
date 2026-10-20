#!/usr/bin/env python3
"""检查项目 2.0 在核桃派上的 Python、串口和摄像头环境。"""

import argparse
import importlib.util
import os
import sys
from pathlib import Path


def module_version(name):
    try:
        module = __import__(name)
    except Exception as exc:  # pragma: no cover - 设备现场错误
        return None, str(exc)
    return getattr(module, "__version__", "installed"), ""


def camera_probe(cv2, device, width, height, fps):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return False
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    ok, frame = cap.read()
    cap.release()
    return bool(ok and frame is not None and getattr(frame, "size", 0) > 0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default="/dev/ttyS2")
    parser.add_argument("--camera-device", default="auto")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    failed = False
    print(f"Python: {sys.version.split()[0]}")

    for name in ("serial", "cv2"):
        version, error = module_version(name)
        if version is None:
            failed = True
            print(f"FAIL {name}: {error}")
        else:
            print(f"OK   {name}: {version}")

    serial_path = Path(args.serial)
    if serial_path.exists():
        print(f"OK   serial device: {serial_path}")
        if not os.access(serial_path, os.R_OK | os.W_OK):
            failed = True
            print("FAIL serial permission: add user to dialout and re-login")
    else:
        failed = True
        print(f"FAIL serial device missing: {serial_path}")

    if importlib.util.find_spec("cv2") is not None:
        import cv2

        candidates = (
            [args.camera_device]
            if args.camera_device != "auto"
            else [f"/dev/video{i}" for i in range(8)]
        )
        found = None
        for device in candidates:
            if Path(device).exists() and camera_probe(
                cv2, device, args.width, args.height, args.fps
            ):
                found = device
                break
        if found:
            print(f"OK   camera: {found}")
        else:
            failed = True
            print("FAIL camera: no readable V4L2 frame")

    print("")
    if failed:
        print("结果：环境未完全就绪，请先处理 FAIL 项。")
        return 1
    print("结果：核桃派运行环境已就绪。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
