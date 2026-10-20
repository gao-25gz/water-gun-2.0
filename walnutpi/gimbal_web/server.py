#!/usr/bin/env python3
"""浏览器版二层云台控制台。

核桃派负责串口和摄像头，电脑只需要用浏览器访问本程序。
"""

import argparse
import json
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

try:
    import cv2
except ImportError:
    cv2 = None


ANGLE_RE = re.compile(r"^A:(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)$")
GIMBAL_RE = re.compile(
    r"\[GIMBAL\]\s+yaw=([-+]?\d+(?:\.\d+)?)\s+"
    r"pitch=([-+]?\d+(?:\.\d+)?)"
)
PITCH_RE = re.compile(
    r"\[PITCH\]\s+raw=([-+]?\d+(?:\.\d+)?)\s+"
    r"relative=([-+]?\d+(?:\.\d+)?)\s+"
    r"comp=([-+]?\d+(?:\.\d+)?)\s+enabled=(\d+)"
)
PITCH_OLD_RE = re.compile(
    r"\[PITCH\]\s+angle=([-+]?\d+(?:\.\d+)?)\s+"
    r"comp=([-+]?\d+(?:\.\d+)?)\s+enabled=(\d+)"
)


class GimbalController:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.serial = None
        self.lock = threading.Lock()
        self.running = True
        self.last_rx = ""
        self.last_tx = ""
        self.last_error = ""
        self.yaw = 0.0
        self.pitch = 0.0
        self.relay_state = False
        self.relay_ack = ""
        self.gimbal_seq = 0
        self.pitch_feedback = None
        self.pitch_raw = None
        self.pitch_compensation = 0.0
        self.comp_enabled = True
        self.logs = deque(maxlen=120)

        self._open_serial()
        threading.Thread(target=self._reader, daemon=True).start()

    @property
    def connected(self):
        return self.serial is not None and self.serial.is_open

    def _open_serial(self):
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
            with self.lock:
                self.last_error = ""
            return True
        except (serial.SerialException, OSError) as exc:
            with self.lock:
                self.last_error = str(exc)
            self.serial = None
            return False

    def _close_serial(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None

    def _reader(self):
        while self.running:
            if not self.connected:
                if not self._open_serial():
                    time.sleep(1.0)
                    continue
            try:
                line = self.serial.readline()
            except (serial.SerialException, OSError) as exc:
                with self.lock:
                    self.last_error = str(exc)
                self._close_serial()
                continue

            if not line:
                continue

            text = line.decode("ascii", errors="replace").replace(
                "\x00", " "
            ).strip()
            if not text:
                continue

            with self.lock:
                self.last_rx = text
                pitch_match = PITCH_RE.search(text)
                if pitch_match:
                    self.pitch_raw = float(pitch_match.group(1))
                    self.pitch_feedback = float(pitch_match.group(2))
                    self.pitch_compensation = float(pitch_match.group(3))
                    self.comp_enabled = bool(int(pitch_match.group(4)))
                else:
                    old_pitch_match = PITCH_OLD_RE.search(text)
                    if old_pitch_match:
                        self.pitch_raw = float(old_pitch_match.group(1))
                        self.pitch_feedback = float(old_pitch_match.group(1))
                        self.pitch_compensation = float(old_pitch_match.group(2))
                        self.comp_enabled = bool(int(old_pitch_match.group(3)))
                    self.logs.append(text)

                if "[RELAY] on" in text:
                    self.relay_state = True
                elif "[RELAY] off" in text:
                    self.relay_state = False
                if "[RELAY] fire_ack pb0=1" in text:
                    self.relay_state = True
                    self.relay_ack = "开火指令已到达 STM32，PB0=高电平"
                elif "[RELAY] off_ack pb0=0" in text:
                    self.relay_state = False
                    self.relay_ack = "继电器关闭，PB0=低电平"

                match = GIMBAL_RE.search(text)
                if match:
                    self.yaw = float(match.group(1))
                    self.pitch = float(match.group(2))
                    self.gimbal_seq += 1

    def send(self, command):
        if not self.connected:
            return False, self.last_error or "串口未连接"

        try:
            with self.lock:
                self.serial.write((command + "\n").encode("ascii"))
                self.serial.flush()
                self.last_tx = command
            return True, ""
        except (serial.SerialException, OSError) as exc:
            with self.lock:
                self.last_error = str(exc)
            self._close_serial()
            return False, str(exc)

    def status(self):
        with self.lock:
            return {
                "connected": self.connected,
                "port": self.port,
                "baud": self.baud,
                "yaw": self.yaw,
                "pitch": self.pitch,
                "gimbal_seq": self.gimbal_seq,
                "pitch_feedback": self.pitch_feedback,
                "pitch_raw": self.pitch_raw,
                "pitch_compensation": self.pitch_compensation,
                "comp_enabled": self.comp_enabled,
                "relay_ack": self.relay_ack,
                "last_rx": self.last_rx,
                "last_tx": self.last_tx,
                "logs": list(self.logs),
                "error": self.last_error,
            }

    def close(self):
        self.running = False
        self._close_serial()


class RelayController:
    def __init__(
        self,
        port="",
        baud=115200,
        on_command="FIRE",
        off_command="OFF",
        controller=None,
    ):
        self.port = port
        self.baud = baud
        self.on_command = on_command
        self.off_command = off_command
        self.controller = controller
        self.serial = None
        self.lock = threading.Lock()
        self.last_tx = ""
        self.last_error = ""
        self.state = False

        if self.controller is None:
            try:
                self.serial = serial.Serial(port, baud, timeout=0.1)
            except serial.SerialException as exc:
                self.last_error = str(exc)

    @property
    def connected(self):
        if self.controller is not None:
            return self.controller.connected
        return self.serial is not None and self.serial.is_open

    def send(self, command, state=None):
        if not self.connected:
            return False, self.last_error or "串口未连接"

        if self.controller is not None:
            ok, error = self.controller.send(command)
            if ok and state is not None:
                with self.lock:
                    self.last_tx = command
                    self.state = bool(state)
            return ok, error

        try:
            with self.lock:
                self.serial.write((command + "\n").encode("ascii"))
                self.serial.flush()
                self.last_tx = command
                if state is not None:
                    self.state = bool(state)
            return True, ""
        except serial.SerialException as exc:
            with self.lock:
                self.last_error = str(exc)
            return False, str(exc)

    def status(self):
        with self.lock:
            state = (
                self.controller.relay_state
                if self.controller is not None
                else self.state
            )
            return {
                "connected": self.connected,
                "port": (
                    self.controller.port
                    if self.controller is not None
                    else self.port
                ),
                "baud": self.baud,
                "state": state,
                "ack": (
                    self.controller.relay_ack
                    if self.controller is not None
                    else ""
                ),
                "last_tx": self.last_tx,
                "error": self.last_error,
            }

    def close(self):
        if (
            self.controller is None
            and self.serial is not None
            and self.serial.is_open
        ):
            self.serial.close()


class CameraStream:
    def __init__(self, device, width, height, fps, quality):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.cap = None
        self.lock = threading.Lock()
        self.last_error = ""
        self.last_fps = 0.0
        self.last_fps_tick = time.monotonic()
        self.frame_count = 0
        self.last_reopen_tick = 0.0

    @property
    def opened(self):
        return self.cap is not None

    def open(self):
        if cv2 is None:
            self.last_error = "未安装 OpenCV，请安装 python3-opencv"
            return False

        self.close()
        candidates = (
            [self.device]
            if self.device != "auto"
            else [f"/dev/video{i}" for i in range(8)]
        )
        self.cap = None
        resolved = None
        for candidate in candidates:
            if not Path(candidate).exists():
                continue
            cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
            cap.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*"MJPG"),
            )
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if cap.isOpened():
                self.cap = cap
                resolved = candidate
                break
            cap.release()

        if self.cap is None:
            self.last_error = f"无法打开摄像头：{self.device}"
            return False

        self.device = resolved
        self.last_error = ""
        self.last_fps = 0.0
        self.last_fps_tick = time.monotonic()
        self.frame_count = 0
        return True

    def get_jpeg(self):
        with self.lock:
            if self.cap is None:
                self._maybe_reopen()
                return None

            ok, frame = self.cap.read()
            if not ok:
                self._maybe_reopen()
                return None

            self.frame_count += 1
            now = time.monotonic()
            if now - self.last_fps_tick >= 1.0:
                self.last_fps = self.frame_count / (now - self.last_fps_tick)
                self.frame_count = 0
                self.last_fps_tick = now

            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.quality],
            )
            if not ok:
                return None
            return encoded.tobytes()

    def _maybe_reopen(self):
        now = time.monotonic()
        if now - self.last_reopen_tick < 2.0:
            return
        self.last_reopen_tick = now
        self.open()

    def status(self):
        return {
            "opened": self.opened,
            "device": self.device,
            "fps": round(self.last_fps, 1),
            "error": self.last_error,
        }

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def valid_command(command):
    if command in {"L", "R", "U", "D", "C", "S", "Z", "N"}:
        return True
    if command.startswith("Y:") or command.startswith("P:"):
        return bool(re.fullmatch(r"[YP]:-?\d+(?:\.\d+)?", command))
    return bool(ANGLE_RE.fullmatch(command))


class RequestHandler(BaseHTTPRequestHandler):
    controller = None
    relay = None
    camera = None
    page = b""

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in {"/", "/index.html"}:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(self.page)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(self.page)
            return

        if path == "/api/status":
            relay_status = (
                self.relay.status()
                if self.relay is not None
                else {
                    "connected": False,
                    "port": None,
                    "baud": None,
                    "state": False,
                    "last_tx": "",
                    "error": "未配置继电器串口",
                }
            )
            self._send_json({
                **self.controller.status(),
                "relay": relay_status,
                "camera": self.camera.status(),
            })
            return

        if path == "/video.mjpg":
            self._stream_camera()
            return

        self._send_json({"error": "找不到页面"}, 404)

    def _stream_camera(self):
        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        last_frame = None
        try:
            while True:
                frame = self.camera.get_jpeg()
                if frame is None or frame == last_frame:
                    time.sleep(0.02)
                    continue
                last_frame = frame
                self.wfile.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: "
                    + str(len(frame)).encode("ascii")
                    + b"\r\n\r\n"
                    + frame
                    + b"\r\n"
                )
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/command":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                data = json.loads(self.rfile.read(length).decode("utf-8"))
                command = str(data.get("command", "")).strip().upper()
            except (ValueError, json.JSONDecodeError):
                self._send_json({"error": "指令格式不正确"}, 400)
                return

            if not valid_command(command):
                self._send_json({"error": "不支持的指令"}, 400)
                return

            ok, error = self.controller.send(command)
            if not ok:
                self._send_json({"error": error}, 503)
                return
            self._send_json({"ok": True, "command": command})
            return

        if path == "/api/relay":
            if self.relay is None:
                self._send_json({"error": "继电器未配置"}, 404)
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
                data = json.loads(self.rfile.read(length).decode("utf-8"))
                action = str(data.get("action", "")).strip().lower()
            except (ValueError, json.JSONDecodeError):
                self._send_json({"error": "指令格式不正确"}, 400)
                return

            if action not in {"on", "off"}:
                self._send_json({"error": "不支持的动作"}, 400)
                return

            if action == "on":
                ok, error = self.relay.send(self.relay.on_command, True)
            else:
                ok, error = self.relay.send(self.relay.off_command, False)

            if not ok:
                self._send_json({"error": error}, 503)
                return
            self._send_json({"ok": True, "action": action})
            return

        self._send_json({"error": "找不到接口"}, 404)

    def log_message(self, fmt, *args):
        return


def main():
    parser = argparse.ArgumentParser(description="二层云台浏览器上位机")
    parser.add_argument("--serial", default="/dev/ttyS2")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--camera-device", default="auto")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--jpeg-quality", type=int, default=70)
    parser.add_argument("--relay-serial", default="")
    parser.add_argument("--relay-baud", type=int, default=115200)
    parser.add_argument("--relay-on-command", default="FIRE")
    parser.add_argument("--relay-off-command", default="OFF")
    args = parser.parse_args()

    page_path = Path(__file__).with_name("index.html")
    RequestHandler.page = page_path.read_bytes()
    controller = GimbalController(args.serial, args.baud)
    relay = None
    if args.relay_serial:
        relay = RelayController(
            args.relay_serial,
            args.relay_baud,
            args.relay_on_command,
            args.relay_off_command,
        )
    else:
        # 继电器现在由同一块 STM32 的 PB0 驱动。
        # 复用已有的 USART2 连接，不再打开第二个串口。
        relay = RelayController(
            on_command=args.relay_on_command,
            off_command=args.relay_off_command,
            controller=controller,
        )

    if relay.connected:
        relay.send(relay.off_command, False)
    camera = CameraStream(
        args.camera_device,
        args.camera_width,
        args.camera_height,
        args.camera_fps,
        max(30, min(95, args.jpeg_quality)),
    )
    camera.open()
    RequestHandler.controller = controller
    RequestHandler.relay = relay
    RequestHandler.camera = camera

    server = ThreadingHTTPServer((args.host, args.port), RequestHandler)
    server.daemon_threads = True
    print(f"网页地址：http://0.0.0.0:{args.port}")
    print(f"串口：{args.serial} @ {args.baud}")
    print(f"摄像头：{args.camera_device}")
    if args.relay_serial:
        print(f"继电器串口：{args.relay_serial} @ {args.relay_baud}")
    else:
        print("继电器：复用 STM32 USART2，PB0 输出")
    if not controller.connected:
        print(f"串口错误：{controller.last_error}")
    if relay is not None and not relay.connected:
        print(f"继电器错误：{relay.last_error}")
    if not camera.opened:
        print(f"摄像头错误：{camera.last_error}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        if relay is not None:
            relay.close()
        camera.close()
        server.server_close()


if __name__ == "__main__":
    main()
