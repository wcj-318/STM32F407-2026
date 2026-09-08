import sensor
import time
import gc
import math


# OpenMV H7 Plus, firmware 5.0.0, QVGA 320x240.
# Stage 1: RGB/LAB pipe localization.
# Stage 2: grayscale ball detection only inside the stabilized pipe ROI.

FRAME_W = 320
FRAME_H = 240

PIPE_SEARCH_ROI = (20, 65, 285, 115)
PIPE_THRESHOLD = (35, 100, -30, 30, -30, 30)
PIPE_L_OFFSET = 18
PIPE_L_MIN_FLOOR = 28
PIPE_L_MIN_CEILING = 82
PIPE_LOCAL_PAD_X = 15
PIPE_LOCAL_PAD_Y = 18
PIPE_PIXELS_MIN = 40
PIPE_AREA_MIN = 80
PIPE_WIDTH_MIN = 70
PIPE_HEIGHT_MIN = 8
PIPE_HEIGHT_MAX = 70
PIPE_RATIO_MIN = 2.5
PIPE_TARGET_HEIGHT = 32
PIPE_MARGIN = 3
PIPE_ALPHA = 0.18
PIPE_HOLD_FRAMES = 15

BALL_GRAY_MIN = 35
BALL_GRAY_MAX = 185
BALL_DARK_OFFSET = 25
BALL_PIXELS_MIN = 8
BALL_AREA_MIN = 20
BALL_AREA_MAX = 1200
BALL_DIAMETER_MIN = 5
BALL_DIAMETER_MAX = 45
BALL_RATIO_MAX = 2.5
BALL_ROUNDNESS_MIN = 0.22

CIRCLE_R_MIN = 4
CIRCLE_R_MAX = 22
CIRCLE_THRESHOLD = 650
CIRCLE_MEAN_MIN = 45
CIRCLE_MEAN_MAX = 180
CIRCLE_MEDIAN_MIN = 40
CIRCLE_MEDIAN_MAX = 170

TRACK_JUMP_MAX = 70
TRACK_HOLD_FRAMES = 12
POSITION_ALPHA_FAST = 0.72
POSITION_ALPHA_SLOW = 0.28
FAST_MOVE_PIXELS = 8
POSITION_DEADBAND = 1.2

EXPOSURE_SCALE = 0.45
PRINT_INTERVAL = 1


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def limit_roi(x, y, w, h):
    x = int(clamp(x, 0, FRAME_W - 1))
    y = int(clamp(y, 0, FRAME_H - 1))
    w = int(clamp(w, 1, FRAME_W - x))
    h = int(clamp(h, 1, FRAME_H - y))
    return (x, y, w, h)


def point_in_roi(x, y, roi, padding=0):
    rx, ry, rw, rh = roi
    return (rx - padding <= x < rx + rw + padding and
            ry - padding <= y < ry + rh + padding)


class PipeTracker:
    def __init__(self):
        self.roi = None
        self.lost = 0

    def update(self, detected):
        if detected is None:
            self.lost += 1
            if self.roi is not None and self.lost <= PIPE_HOLD_FRAMES:
                return self.roi
            self.roi = None
            return None

        self.lost = 0
        if self.roi is None:
            self.roi = detected
            return self.roi

        ox, oy, ow, oh = self.roi
        nx, ny, nw, nh = detected
        self.roi = limit_roi(
            ox + (nx - ox) * PIPE_ALPHA,
            oy + (ny - oy) * PIPE_ALPHA,
            ow + (nw - ow) * PIPE_ALPHA,
            oh + (nh - oh) * PIPE_ALPHA
        )
        return self.roi


class BallTracker:
    def __init__(self):
        self.x = None
        self.y = None
        self.r = None
        self.vx = 0.0
        self.vy = 0.0
        self.lost = 0

    def reset(self):
        self.__init__()

    def update(self, detection, roi):
        if self.x is not None and not point_in_roi(self.x, self.y, roi, 4):
            self.reset()

        if detection is None:
            self.lost += 1
            if self.x is None or self.lost > TRACK_HOLD_FRAMES:
                self.reset()
                return None

            self.x = clamp(self.x + self.vx, roi[0], roi[0] + roi[2] - 1)
            self.y = clamp(self.y + self.vy, roi[1], roi[1] + roi[3] - 1)
            self.vx *= 0.82
            self.vy *= 0.82
            return (int(self.x), int(self.y), int(self.r))

        nx = float(detection["x"])
        ny = float(detection["y"])
        nr = float(detection["r"])

        if self.x is None:
            self.x, self.y, self.r = nx, ny, nr
            self.lost = 0
            return (int(self.x), int(self.y), int(self.r))

        dx = nx - self.x
        dy = ny - self.y
        distance = math.sqrt(dx * dx + dy * dy)
        if distance > TRACK_JUMP_MAX:
            self.lost += 1
            return (int(self.x), int(self.y), int(self.r))

        alpha = POSITION_ALPHA_FAST if distance >= FAST_MOVE_PIXELS else POSITION_ALPHA_SLOW
        if abs(dx) > POSITION_DEADBAND:
            self.x += dx * alpha
        if abs(dy) > POSITION_DEADBAND:
            self.y += dy * alpha
        self.r += (nr - self.r) * 0.25
        self.vx = self.vx * 0.45 + dx * 0.55
        self.vy = self.vy * 0.45 + dy * 0.55
        self.lost = 0
        return (int(self.x), int(self.y), int(self.r))


def detect_pipe(img, previous_roi=None):
    if previous_roi is None:
        search_roi = PIPE_SEARCH_ROI
    else:
        px, py, pw, ph = previous_roi
        search_roi = limit_roi(
            px - PIPE_LOCAL_PAD_X,
            py - PIPE_LOCAL_PAD_Y,
            pw + PIPE_LOCAL_PAD_X * 2,
            ph + PIPE_LOCAL_PAD_Y * 2
        )

    light_stats = img.get_statistics(roi=search_roi)
    dynamic_l_min = int(clamp(
        light_stats.uq - PIPE_L_OFFSET,
        PIPE_L_MIN_FLOOR,
        PIPE_L_MIN_CEILING
    ))
    dynamic_threshold = (dynamic_l_min, 100, -30, 30, -30, 30)

    blobs = img.find_blobs(
        [dynamic_threshold],
        roi=search_roi,
        pixels_threshold=PIPE_PIXELS_MIN,
        area_threshold=PIPE_AREA_MIN,
        merge=True,
        margin=PIPE_MARGIN
    )

    # Static threshold is a fallback for scenes whose histogram is dominated
    # by a bright or dark object outside the pipe.
    if not blobs:
        blobs = img.find_blobs(
            [PIPE_THRESHOLD],
            roi=search_roi,
            pixels_threshold=PIPE_PIXELS_MIN,
            area_threshold=PIPE_AREA_MIN,
            merge=True,
            margin=PIPE_MARGIN
        )

    best = None
    best_score = None
    for blob in blobs:
        w = blob.w
        h = blob.h
        if h <= 0:
            continue
        ratio = w / h
        if not (PIPE_WIDTH_MIN <= w and
                PIPE_HEIGHT_MIN <= h <= PIPE_HEIGHT_MAX and
                ratio >= PIPE_RATIO_MIN):
            continue
        score = w * 4 - h * 2
        if best_score is None or score > best_score:
            best = blob
            best_score = score

    if best is None:
        return None

    x, y, w, h = best.x, best.y, best.w, best.h
    if h > PIPE_TARGET_HEIGHT:
        y += h - PIPE_TARGET_HEIGHT
        h = PIPE_TARGET_HEIGHT
    return limit_roi(x + 2, y + 2, w - 4, h - 4)


def circle_value(circle, index):
    return int(circle[index])


def circle_strength(circle):
    return (circle_value(circle, 3) * 100) // max(1, circle_value(circle, 2))


def circle_has_ball_brightness(gray, circle):
    cx = circle_value(circle, 0)
    cy = circle_value(circle, 1)
    r = circle_value(circle, 2)
    half = max(2, int(r * 0.55))
    roi = limit_roi(cx - half, cy - half, half * 2 + 1, half * 2 + 1)
    stats = gray.get_statistics(roi=roi)
    return (CIRCLE_MEAN_MIN <= stats.mean <= CIRCLE_MEAN_MAX and
            CIRCLE_MEDIAN_MIN <= stats.median <= CIRCLE_MEDIAN_MAX)


def find_circles(gray, roi):
    circles = gray.find_circles(
        roi=roi,
        threshold=CIRCLE_THRESHOLD,
        x_stride=2,
        y_stride=2,
        r_min=CIRCLE_R_MIN,
        r_max=CIRCLE_R_MAX,
        r_step=1,
        x_margin=8,
        y_margin=8,
        r_margin=8
    )
    return [c for c in circles if circle_has_ball_brightness(gray, c)]


def detect_ball(img, pipe_roi, tracker):
    gray = img.to_grayscale(copy=True)
    gray.gaussian(1)
    stats = gray.get_statistics(roi=pipe_roi)
    dark_max = int(clamp(stats.mean - BALL_DARK_OFFSET, BALL_GRAY_MIN + 1, BALL_GRAY_MAX))

    blobs = gray.find_blobs(
        [(BALL_GRAY_MIN, dark_max)],
        roi=pipe_roi,
        pixels_threshold=BALL_PIXELS_MIN,
        area_threshold=BALL_PIXELS_MIN,
        merge=False
    )

    candidates = []
    for blob in blobs:
        w, h, area = blob.w, blob.h, blob.area
        if w <= 0 or h <= 0:
            continue
        ratio = max(w, h) / min(w, h)
        if not (BALL_AREA_MIN <= area <= BALL_AREA_MAX and
                BALL_DIAMETER_MIN <= min(w, h) and
                max(w, h) <= BALL_DIAMETER_MAX and
                ratio <= BALL_RATIO_MAX and
                blob.roundness >= BALL_ROUNDNESS_MIN):
            continue

        distance_penalty = 0
        if tracker.x is not None:
            dx = blob.cx - (tracker.x + tracker.vx)
            dy = blob.cy - (tracker.y + tracker.vy)
            distance_penalty = (dx * dx + dy * dy) * 0.02
        candidates.append({
            "x": blob.cx,
            "y": blob.cy,
            "r": int(clamp((w + h) * 0.25, CIRCLE_R_MIN, CIRCLE_R_MAX)),
            "score": blob.roundness * 120 + min(w, h) * 2 - distance_penalty
        })

    circles = find_circles(gray, pipe_roi)
    if circles:
        if tracker.x is None:
            circle = max(circles, key=circle_strength)
        else:
            px = tracker.x + tracker.vx
            py = tracker.y + tracker.vy
            circle = max(
                circles,
                key=lambda c: circle_strength(c) - 0.02 * (
                    (circle_value(c, 0) - px) ** 2 +
                    (circle_value(c, 1) - py) ** 2
                )
            )
        return {
            "x": circle_value(circle, 0),
            "y": circle_value(circle, 1),
            "r": circle_value(circle, 2),
            "source": "circle"
        }

    if not candidates:
        return None

    best = max(candidates, key=lambda item: item["score"])
    best["source"] = "blob"
    return best


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
sensor.skip_frames(time=2000)

try:
    gain_db = sensor.get_gain_db()
    exposure_us = sensor.get_exposure_us()
    sensor.set_auto_gain(False, gain_db=gain_db)
    sensor.set_auto_exposure(
        False,
        exposure_us=max(100, int(exposure_us * EXPOSURE_SCALE))
    )
except Exception:
    sensor.set_auto_gain(False)
    sensor.set_auto_exposure(False)

sensor.set_auto_whitebal(False)
sensor.skip_frames(time=300)

clock = time.clock()
pipe_tracker = PipeTracker()
ball_tracker = BallTracker()
frame_index = 0

print("OpenMV 5 hybrid pipe/ball tracker started")

while True:
    clock.tick()
    frame_index += 1
    img = sensor.snapshot()

    detected_pipe = detect_pipe(img, pipe_tracker.roi)
    pipe_roi = pipe_tracker.update(detected_pipe)

    tracked = None
    if pipe_roi is not None:
        detection = detect_ball(img, pipe_roi, ball_tracker)
        tracked = ball_tracker.update(detection, pipe_roi)
        img.draw_rectangle(pipe_roi, color=(0, 150, 255), thickness=1)
    else:
        ball_tracker.reset()

    if tracked is not None:
        x, y, radius = tracked
        img.draw_circle((x, y, radius), color=(0, 255, 0), thickness=2)
        img.draw_cross((x, y), color=(255, 0, 0), size=5, thickness=1)
        if frame_index % PRINT_INTERVAL == 0:
            print("BALL,%d,%d,%d" % (x, y, radius))
    elif frame_index % 10 == 0:
        print("NO_BALL")

    img.draw_string(
        (2, 2),
        "FPS:%.1f" % clock.fps(),
        color=(255, 255, 0),
        scale=1
    )

    if frame_index & 0x3F == 0:
        gc.collect()
