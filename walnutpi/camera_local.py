#!/usr/bin/env python3
import time
import cv2

DEVICE = '/dev/video1'
WIDTH, HEIGHT, FPS = 640, 480, 60

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    raise RuntimeError(f'无法打开摄像头: {DEVICE}')

last = time.monotonic()
count = 0
shown_fps = 0.0

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    count += 1
    now = time.monotonic()
    if now - last >= 1.0:
        shown_fps = count / (now - last)
        count = 0
        last = now

    cv2.putText(frame, f'{shown_fps:.1f} FPS', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow('WalnutPi Camera - press q to quit', frame)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
