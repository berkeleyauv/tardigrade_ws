"""Pure-CV gate detection pipeline, adapted from UR-B-Perception/ClassicalOrientation.py.

No ROS, no GUI, no matplotlib -- just numpy/cv2 functions so it can be unit
tested on still images and reused by the ROS node.

The original script needed a YOLO/hand-drawn gate bounding box. Here we find
the gate ROI automatically using the RED panels (LAB a-channel), since red is
the most distinctive color underwater. Post detection and alignment math are
unchanged from ClassicalOrientation.py.
"""

import cv2
import numpy as np

# Defaults match ClassicalOrientation.py -- tune with the pool footage.
L_THRESHOLD = 60
A_THRESHOLD = 135
MIN_BLOB_AREA = 50
TOLERANCE = 0.04
BLACK_PERCENTILE = 15.0

# ROI finder tuning (post-candidate approach -- see find_gate_roi)
ROI_DARK_PERCENTILE = 8.0     # darkest N% of pixels = post candidates
ROI_MIN_POST_HEIGHT = 0.12    # post must be at least this fraction of image height
ROI_MIN_POST_ASPECT = 2.5     # post bounding box h/w must exceed this
ROI_MIN_PAIR_SEP = 0.10       # posts must be separated by this fraction of image width
ROI_MAX_PAIR_SEP = 0.90
ROI_PAD_FRAC = 0.10           # pad the final box by this fraction


def enhance_underwater(img):
    """Per-channel white balance + CLAHE on L channel. ~few ms at 640px."""
    result = img.astype(np.float32)
    for i in range(3):
        ch = result[:, :, i]
        lo, hi = ch.min(), ch.max()
        if hi > lo:
            result[:, :, i] = (ch - lo) / (hi - lo) * 255
    result = result.astype(np.uint8)
    lab = cv2.cvtColor(result, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    lab = cv2.merge((clahe.apply(l), a, b))
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


def segment_black_lab(crop, percentile=BLACK_PERCENTILE):
    lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
    l_ch = lab[:, :, 0]
    dynamic_threshold = np.percentile(l_ch, percentile)
    _, mask = cv2.threshold(l_ch, dynamic_threshold, 255, cv2.THRESH_BINARY_INV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return mask


def segment_red_lab(img, a_threshold=A_THRESHOLD):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab = cv2.merge((clahe.apply(l), a, b))
    a_ch = lab[:, :, 1]
    _, mask = cv2.threshold(a_ch, a_threshold, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return mask


def largest_blob(mask, min_area=MIN_BLOB_AREA):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return None
    best = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(best)
    return dict(
        area=cv2.contourArea(best),
        cx=x + w / 2.0, cy=y + h / 2.0,
        x=x, y=y, w=w, h=h,
    )


def find_post_candidates(img, percentile=ROI_DARK_PERCENTILE):
    """Tall, thin, dark blobs -- likely gate posts (panels look dark/red-brown
    underwater, water and pool walls are bright blue)."""
    ih, iw = img.shape[:2]
    mask = segment_black_lab(img, percentile)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if h < ih * ROI_MIN_POST_HEIGHT:
            continue
        if h < w * ROI_MIN_POST_ASPECT:
            continue
        area = cv2.contourArea(c)
        # Mean stroke width: less sensitive to reflections/blur inflating the
        # bounding box than the raw bbox width. Used for the yaw ratio.
        mean_w = area / float(max(h, 1))
        candidates.append((x, y, w, h, mean_w))
    # Tallest first
    candidates.sort(key=lambda b: b[3], reverse=True)
    return candidates


def find_gate_elements(img, percentile=ROI_DARK_PERCENTILE):
    """Locate the gate by pairing dark vertical post candidates.

    Replaces the YOLO box / hand-drawn ROI in the original script. Returns
    dict(roi=(x, y, w, h), left_box=..., right_box=...) in full-image pixels
    (left_box/right_box are None when not found), or None if no gate evidence.
    Swap this for a YOLO detector later without touching anything else.
    """
    ih, iw = img.shape[:2]
    candidates = find_post_candidates(img, percentile)
    if not candidates:
        return None

    # Try to pair the tallest candidate with another one that is horizontally
    # separated like a gate and vertically overlapping.
    best_pair = None
    for i in range(min(len(candidates), 4)):
        for j in range(i + 1, min(len(candidates), 5)):
            a, b = candidates[i], candidates[j]
            sep = abs((a[0] + a[2] / 2.0) - (b[0] + b[2] / 2.0))
            if not (iw * ROI_MIN_PAIR_SEP <= sep <= iw * ROI_MAX_PAIR_SEP):
                continue
            # vertical overlap of the two boxes
            top = max(a[1], b[1])
            bot = min(a[1] + a[3], b[1] + b[3])
            if bot - top < 0.3 * min(a[3], b[3]):
                continue
            if best_pair is None or sep > best_pair[0]:
                best_pair = (sep, a, b)

    left_box = right_box = None
    if best_pair is not None:
        _, a, b = best_pair
        left_box, right_box = sorted([a, b], key=lambda box: box[0])
        boxes = [a, b]
    else:
        # Single post visible (gate partially in frame): build a ROI around it
        # so alignment still gets a lateral cue.
        boxes = [candidates[0]]

    x0 = min(b[0] for b in boxes)
    y0 = min(b[1] for b in boxes)
    x1 = max(b[0] + b[2] for b in boxes)
    y1 = max(b[1] + b[3] for b in boxes)

    pad_x = max(int((x1 - x0) * ROI_PAD_FRAC), 4)
    pad_y = max(int((y1 - y0) * ROI_PAD_FRAC), 4)
    x0 = max(0, x0 - pad_x)
    y0 = max(0, y0 - pad_y)
    x1 = min(iw, x1 + pad_x)
    y1 = min(ih, y1 + pad_y)

    w, h = x1 - x0, y1 - y0
    if w <= 0 or h <= 0:
        return None
    return dict(roi=(x0, y0, w, h), left_box=left_box, right_box=right_box)


def detect_posts(image, gate_roi, percentile=BLACK_PERCENTILE):
    """Find the left/right black panels inside the gate ROI.

    Gate layout (RoboSub 2026): left post has BLACK on top, right post has
    BLACK on bottom -> search upper-left and lower-right quadrants.
    """
    gx, gy, gw, gh = gate_roi
    crop = image[gy:gy + gh, gx:gx + gw]
    if crop.size == 0:
        return None, None

    black_mask = segment_black_lab(crop, percentile)
    divider_y = gh // 2
    mid_x = gw // 2

    left_zone = np.zeros_like(black_mask)
    left_zone[:divider_y, :mid_x] = 255
    right_zone = np.zeros_like(black_mask)
    right_zone[divider_y:, mid_x:] = 255

    left_stats = largest_blob(cv2.bitwise_and(black_mask, left_zone))
    right_stats = largest_blob(cv2.bitwise_and(black_mask, right_zone))

    def translate(s):
        return dict(s, img_cx=s["cx"] + gx, img_cy=s["cy"] + gy,
                    img_x=s["x"] + gx, img_y=s["y"] + gy)

    if left_stats:
        left_stats = translate(left_stats)
    if right_stats:
        right_stats = translate(right_stats)
    return left_stats, right_stats


def compute_alignment(left_stats, right_stats, image_width, gate_roi):
    """Same math as ClassicalOrientation.compute_alignment, numeric output.

    Returns dict with:
      yaw_signal   -- (W_R / W_L) - 1, or None if a post is missing
      lateral_norm -- gate center offset from image center in [-1, 1]
    """
    eps = 1e-6
    cx = image_width / 2.0

    if left_stats and right_stats:
        # Prefer mean stroke width when available (robust to reflections)
        w_l = max(left_stats.get("mean_w", left_stats["w"]), eps)
        w_r = max(right_stats.get("mean_w", right_stats["w"]), eps)
        yaw_signal = (w_r / w_l) - 1.0
    else:
        yaw_signal = None

    if gate_roi:
        gx, _, gw, _ = gate_roi
        gate_mid_x = gx + gw / 2.0
    elif left_stats and right_stats:
        gate_mid_x = (left_stats["img_cx"] + right_stats["img_cx"]) / 2.0
    elif left_stats:
        gate_mid_x = left_stats["img_cx"]
    elif right_stats:
        gate_mid_x = right_stats["img_cx"]
    else:
        gate_mid_x = cx

    lateral_norm = float(np.clip((gate_mid_x - cx) / cx, -1.0, 1.0))
    return dict(yaw_signal=yaw_signal, lateral_norm=lateral_norm,
                gate_mid_x=gate_mid_x)


def analyze_frame(bgr, enhance=True, percentile=BLACK_PERCENTILE,
                  a_threshold=A_THRESHOLD):
    """Full pipeline on one BGR frame.

    Returns None if no gate found, else a dict:
      detected, both_posts, yaw_signal, lateral_norm, gate_width_frac, roi
    """
    img = enhance_underwater(bgr) if enhance else bgr
    elements = find_gate_elements(img)
    if elements is None:
        return None

    roi = elements["roi"]

    def box_to_stats(box):
        if box is None:
            return None
        x, y, w, h, mean_w = box
        return dict(area=float(w * h), x=x, y=y, w=w, h=h, mean_w=mean_w,
                    cx=x + w / 2.0, cy=y + h / 2.0,
                    img_x=x, img_y=y,
                    img_cx=x + w / 2.0, img_cy=y + h / 2.0)

    # Use the paired post boxes directly for the perspective yaw cue --
    # more robust than re-segmenting quadrants (surface reflections are dark
    # too and pollute the quadrant search).
    left_stats = box_to_stats(elements["left_box"])
    right_stats = box_to_stats(elements["right_box"])

    aln = compute_alignment(left_stats, right_stats, img.shape[1], roi)

    return dict(
        detected=True,
        both_posts=bool(left_stats and right_stats),
        yaw_signal=aln["yaw_signal"],
        lateral_norm=aln["lateral_norm"],
        gate_width_frac=roi[2] / float(img.shape[1]),
        roi=roi,
        left_stats=left_stats,
        right_stats=right_stats,
        enhanced=img,
    )


def draw_debug(result, frame):
    """Annotate a frame with detection output for the debug image topic."""
    out = frame.copy()
    if result is None:
        cv2.putText(out, "NO GATE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 0, 255), 2, cv2.LINE_AA)
        return out

    gx, gy, gw, gh = result["roi"]
    cv2.rectangle(out, (gx, gy), (gx + gw, gy + gh), (0, 165, 255), 2)
    for stats, color in [(result["left_stats"], (50, 220, 50)),
                         (result["right_stats"], (50, 50, 220))]:
        if stats:
            ix, iy = int(stats["img_x"]), int(stats["img_y"])
            cv2.rectangle(out, (ix, iy),
                          (ix + int(stats["w"]), iy + int(stats["h"])),
                          color, 2)

    ys = result["yaw_signal"]
    txt = "yaw={} lat={:+.2f} w={:.2f}".format(
        "{:+.3f}".format(ys) if ys is not None else "N/A",
        result["lateral_norm"], result["gate_width_frac"])
    cv2.putText(out, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2, cv2.LINE_AA)
    return out
