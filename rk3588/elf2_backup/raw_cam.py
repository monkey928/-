import cv2, numpy as np, sys
cap = cv2.VideoCapture("/dev/video21")
if not cap.isOpened():
    sys.exit(1)

cv2.namedWindow("Cam", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Cam", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

fc = 0
while fc < 50:  # Run for 50 frames then stop
    ok, frame = cap.read()
    if not ok: continue
    fc += 1
    h, w = frame.shape[:2]
    cv2.putText(frame, f"{w}x{h} F:{fc}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Cam", frame)
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()
print(f"Done: {fc} frames, size={w}x{h}")
