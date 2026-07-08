import cv2, numpy as np, os, glob

DATASET = "/home/elf/dataset"
images = sorted(glob.glob(DATASET + "/*.jpg"))
print("Processing", len(images), "images...")

labeled = 0
for path in images:
    frame = cv2.imread(path)
    if frame is None:
        continue
    # Resize to match live detection resolution
    frame = cv2.resize(frame, (640, 480))
    h, w = frame.shape[:2]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 180), (180, 50, 255))
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [(c, cv2.contourArea(c)) for c in cnts if cv2.contourArea(c) > 100]

    txt_path = path.replace(".jpg", ".txt")
    lines = []
    for c, area in valid:
        x, y, bw, bh = cv2.boundingRect(c)
        cx = (x + bw / 2.0) / w
        cy = (y + bh / 2.0) / h
        nw = bw / float(w)
        nh = bh / float(h)
        lines.append("0 %.6f %.6f %.6f %.6f" % (cx, cy, nw, nh))

    if lines:
        with open(txt_path, "w") as f:
            f.write("
".join(lines) + "
")
        labeled += 1
    else:
        # Write empty file for images without detection
        open(txt_path, "w").close()

    if labeled % 50 == 0:
        print("  %d/300 labeled..." % labeled)

print("Done! %d/%d images have labels" % (labeled, len(images)))
