#!/usr/bin/env python3
"""Dataset collection - headless, no display"""
import cv2, os, time

SAVE_DIR = "/home/elf/dataset"
TOTAL = 300
os.makedirs(SAVE_DIR, exist_ok=True)
existing = len([f for f in os.listdir(SAVE_DIR) if f.endswith('.jpg')])

cap = cv2.VideoCapture("/dev/video21")
if not cap.isOpened():
    print("ERROR: cannot open camera")
    exit(1)

print(f"Camera OK. Starting from {existing}, target {TOTAL}")
print("Move paper ball - different angles, distances, lighting")
print("Press Ctrl+C to stop
")

fc = existing
last_save = 0
try:
    while fc < TOTAL:
        ok, frame = cap.read()
        if not ok: continue
        now = time.time()
        if now - last_save >= 0.5:
            path = os.path.join(SAVE_DIR, f"paper_{fc:04d}.jpg")
            cv2.imwrite(path, frame)
            fc += 1
            last_save = now
            if fc % 25 == 0:
                print(f"  [{fc}/{TOTAL}] saved")
except KeyboardInterrupt:
    pass

cap.release()
print(f"
Done! {fc} images in {SAVE_DIR}")
