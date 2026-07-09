"""K230 CanMV tic-tac-toe recognition via the largest black grid blob.
Single-file deployment — vision_core_rgb helpers are inlined below.

Algorithm: black threshold -> connected blobs -> largest rotated rectangle -> 3x3 grid.
"""

import time, math, os, gc, sys

from media.sensor import *
from media.display import *
from media.media import *


# ====================================================================
# vision_core_rgb helpers (inlined for single-file deployment)
# ====================================================================

EMPTY = 0
WHITE = 1
BLACK = 2
UNKNOWN = -1


def distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def order_corners(points):
    """Return top-left, top-right, bottom-right, bottom-left."""
    if len(points) != 4:
        raise ValueError("exactly four corners are required")
    pts = [(int(p[0]), int(p[1])) for p in points]
    tl = pts[0]
    tr = pts[0]
    br = pts[0]
    bl = pts[0]
    for point in pts[1:]:
        if point[0] + point[1] < tl[0] + tl[1]:
            tl = point
        if point[0] - point[1] > tr[0] - tr[1]:
            tr = point
        if point[0] + point[1] > br[0] + br[1]:
            br = point
        if point[0] - point[1] < bl[0] - bl[1]:
            bl = point
    ordered = [tl, tr, br, bl]
    for i in range(4):
        for j in range(i + 1, 4):
            if ordered[i] == ordered[j]:
                raise ValueError("corners cannot be ordered uniquely")
    return ordered


def bilinear_point(corners, u, v):
    tl, tr, br, bl = corners
    x = ((1.0 - u) * (1.0 - v) * tl[0] +
         u * (1.0 - v) * tr[0] +
         u * v * br[0] +
         (1.0 - u) * v * bl[0])
    y = ((1.0 - u) * (1.0 - v) * tl[1] +
         u * (1.0 - v) * tr[1] +
         u * v * br[1] +
         (1.0 - u) * v * bl[1])
    return int(round(x)), int(round(y))


def make_grid_cells(board_corners):
    cells = []
    for row in range(3):
        for col in range(3):
            u0 = col / 3.0
            u1 = (col + 1) / 3.0
            v0 = row / 3.0
            v1 = (row + 1) / 3.0
            cells.append([
                bilinear_point(board_corners, u0, v0),
                bilinear_point(board_corners, u1, v0),
                bilinear_point(board_corners, u1, v1),
                bilinear_point(board_corners, u0, v1),
            ])
    return cells


def quad_center(quad):
    x = 0
    y = 0
    for point in quad:
        x += point[0]
        y += point[1]
    return int(round(x / 4.0)), int(round(y / 4.0))


def quad_area(quad):
    total = 0
    for i in range(4):
        a = quad[i]
        b = quad[(i + 1) % 4]
        total += a[0] * b[1] - b[0] * a[1]
    return abs(total) * 0.5


def is_board_blob_geometry(corners, density, image_area, limits):
    """Check whether a blob's rotated geometry can be the complete grid."""
    if image_area <= 0:
        return False
    area_ratio = quad_area(corners) / image_area
    if (area_ratio < limits["min_area_ratio"] or
            area_ratio > limits["max_area_ratio"]):
        return False
    top = distance(corners[0], corners[1])
    right = distance(corners[1], corners[2])
    bottom = distance(corners[2], corners[3])
    left = distance(corners[3], corners[0])
    height = (left + right) * 0.5
    if height < 1:
        return False
    side_ratio = ((top + bottom) * 0.5) / height
    if (side_ratio < limits["min_side_ratio"] or
            side_ratio > limits["max_side_ratio"]):
        return False
    return (limits["min_density"] <= density <=
            limits["max_density"])


def point_in_quad(point, quad):
    """Convex quadrilateral containment including its boundary."""
    sign = 0
    for i in range(4):
        a = quad[i]
        b = quad[(i + 1) % 4]
        cross = ((b[0] - a[0]) * (point[1] - a[1]) -
                 (b[1] - a[1]) * (point[0] - a[0]))
        if cross == 0:
            continue
        current = 1 if cross > 0 else -1
        if sign == 0:
            sign = current
        elif sign != current:
            return False
    return True


def _cell_match_radius(cell, ratio):
    top = distance(cell[0], cell[1])
    right = distance(cell[1], cell[2])
    bottom = distance(cell[2], cell[3])
    left = distance(cell[3], cell[0])
    return min((top + bottom) * 0.5, (left + right) * 0.5) * ratio


def match_small_cells(board_corners, detected_quads, max_distance_ratio=0.45):
    """Match detected small rectangles uniquely to projected row-major cells."""
    expected = make_grid_cells(board_corners)
    result = [cell[:] for cell in expected]
    flags = [False] * 9
    used = [False] * len(detected_quads)
    centers = [quad_center(quad) for quad in detected_quads]
    for cell_index in range(9):
        expected_center = quad_center(expected[cell_index])
        max_distance = _cell_match_radius(expected[cell_index],
                                          max_distance_ratio)
        best_index = -1
        best_distance = max_distance + 1.0
        for detected_index in range(len(detected_quads)):
            if used[detected_index]:
                continue
            current_distance = distance(expected_center,
                                        centers[detected_index])
            if current_distance <= max_distance and current_distance < best_distance:
                best_index = detected_index
                best_distance = current_distance
        if best_index >= 0:
            result[cell_index] = order_corners(detected_quads[best_index])
            flags[cell_index] = True
            used[best_index] = True
    return result, flags


def incircle_geometry(cell_quad, radius_scale=0.35):
    """Return a conservative projected-cell center and circular sample radius."""
    center = quad_center(cell_quad)
    top = distance(cell_quad[0], cell_quad[1])
    right = distance(cell_quad[1], cell_quad[2])
    bottom = distance(cell_quad[2], cell_quad[3])
    left = distance(cell_quad[3], cell_quad[0])
    width = (top + bottom) * 0.5
    height = (left + right) * 0.5
    radius = max(2, int(round(min(width, height) * radius_scale)))
    return center, radius


def circle_rgb_mean(image, cx, cy, radius, step=2):
    """Average RGB pixels inside a circle without allocating an image mask."""
    if radius < 1 or step < 1:
        return None, 0
    x_min = max(0, cx - radius)
    x_max = min(image.width() - 1, cx + radius)
    y_min = max(0, cy - radius)
    y_max = min(image.height() - 1, cy + radius)
    radius_squared = radius * radius
    red_sum = 0
    green_sum = 0
    blue_sum = 0
    count = 0
    y = y_min
    while y <= y_max:
        x = x_min
        while x <= x_max:
            dx = x - cx
            dy = y - cy
            if dx * dx + dy * dy <= radius_squared:
                pixel = image.get_pixel(x, y)
                if isinstance(pixel, tuple):
                    red_sum += pixel[0]
                    green_sum += pixel[1]
                    blue_sum += pixel[2]
                else:
                    red_sum += pixel
                    green_sum += pixel
                    blue_sum += pixel
                count += 1
            x += step
        y += step
    if count == 0:
        return None, 0
    return (int(round(red_sum / count)),
            int(round(green_sum / count)),
            int(round(blue_sum / count))), count


def classify_by_diff(cur_rgb, ref_rgb):
    """Compare current RGB against reference; return (state, diff_sum)."""
    if cur_rgb is None or ref_rgb is None:
        return UNKNOWN, 0
    dr = abs(cur_rgb[0] - ref_rgb[0])
    dg = abs(cur_rgb[1] - ref_rgb[1])
    db = abs(cur_rgb[2] - ref_rgb[2])
    diff = dr + dg + db

    if diff < DIFF_THRESHOLD:
        return EMPTY, diff

    cur_bright = cur_rgb[0] + cur_rgb[1] + cur_rgb[2]
    ref_bright = ref_rgb[0] + ref_rgb[1] + ref_rgb[2]
    if ref_bright > 0 and cur_bright < ref_bright * DARKEN_RATIO:
        return BLACK, diff

    return WHITE, diff


def capture_reference_rgb(img, board_corners):
    """Sample all 9 cells and return list of (r,g,b) tuples (no filtering)."""
    refs = []
    cells = make_grid_cells(board_corners)
    for cell in cells:
        center, radius = incircle_geometry(cell, INCIRCLE_RADIUS_SCALE)
        rgb, count = circle_rgb_mean(img, center[0], center[1], radius,
                                     RGB_SAMPLE_STEP)
        if count < MIN_RGB_SAMPLES:
            rgb = None
        refs.append(rgb)
    return refs


class BoardDebouncer:
    def __init__(self, stable_frames=5):
        if stable_frames < 1:
            raise ValueError("stable_frames must be positive")
        self.stable_frames = stable_frames
        self._candidate = None
        self._count = 0
        self._stable = None

    def reset(self):
        self._candidate = None
        self._count = 0
        self._stable = None

    def current(self):
        return None if self._stable is None else self._stable[:]

    def update(self, board):
        board = list(board)
        if len(board) != 9:
            raise ValueError("board must contain nine cells")
        if UNKNOWN in board:
            self._candidate = None
            self._count = 0
            return self.current()
        if self._candidate == board:
            self._count += 1
        else:
            self._candidate = board[:]
            self._count = 1
        if self._count >= self.stable_frames:
            self._stable = self._candidate[:]
        return self.current()


class RGBMedianFilter:
    """Small per-cell temporal median filter for RGB measurements."""

    def __init__(self, cell_count=9, window=5):
        if cell_count < 1 or window < 1:
            raise ValueError("cell_count and window must be positive")
        self.cell_count = cell_count
        self.window = window
        self._history = [[] for _ in range(cell_count)]

    def reset(self):
        self._history = [[] for _ in range(self.cell_count)]

    @staticmethod
    def _median(values):
        ordered = sorted(values)
        return ordered[len(ordered) // 2]

    def update(self, rgb_values):
        if len(rgb_values) != self.cell_count:
            raise ValueError("unexpected RGB cell count")
        result = []
        for index in range(self.cell_count):
            rgb = rgb_values[index]
            history = self._history[index]
            if rgb is not None:
                history.append((int(rgb[0]), int(rgb[1]), int(rgb[2])))
                if len(history) > self.window:
                    history.pop(0)
            if not history:
                result.append(None)
                continue
            reds = [item[0] for item in history]
            greens = [item[1] for item in history]
            blues = [item[2] for item in history]
            result.append((self._median(reds), self._median(greens),
                           self._median(blues)))
        return result


# ======================== User calibration area ========================
# Grayscale threshold for black grid lines AFTER adaptive histogram equalization.
# histeq normalizes local lighting — highlights get pushed down, shadows lifted.
GRAY_GRID_THRESHOLD = (0, 80)  # dark pixel range on equalized image

BLOB_PIXELS_THRESHOLD = 320
BLOB_AREA_THRESHOLD = 3200
BLOB_MERGE_MARGIN = 8

BOARD_LIMITS = {
    "min_area_ratio": 0.08,
    "max_area_ratio": 0.80,
    "min_side_ratio": 0.50,
    "max_side_ratio": 1.80,
    # Thin connected grid lines have low density; solid objects are rejected.
    "min_density": 0.01,
    "max_density": 0.55,
}

LOCK_STABLE_FRAMES = 6
LOCK_CORNER_DISTANCE = 24.0
MAX_TRACK_JUMP = 110.0
UNLOCK_AFTER_MISSES = 10
EXPOSURE_SETTLE_FRAMES = 20

INCIRCLE_RADIUS_SCALE = 0.28
RGB_SAMPLE_STEP = 4
MIN_RGB_SAMPLES = 12
BOARD_STABLE_FRAMES = 5
RGB_FILTER_WINDOW = 5

# ---- Background subtraction parameters ----
DIFF_THRESHOLD = 60          # Sum of |dR|+|dG|+|dB|, below = EMPTY
DARKEN_RATIO = 0.80          # cur_brightness/ref_brightness < ratio = BLACK
REFERENCE_FRAMES = 3         # Frames to median-average for reference capture
AUTO_REF_FRAMES = 30         # Consecutive all-empty frames before auto re-ref
REF_EMA_ALPHA = 0.05         # EMA speed for auto reference update (0=disabled)

PRINT_EVERY = 20
# ======================================================================

WIDTH = 640
HEIGHT = 480


def corners_close(first, second, maximum_average_distance):
    if first is None or second is None:
        return False
    total = 0.0
    for a, b in zip(first, second):
        total += distance(a, b)
    return total / 4.0 <= maximum_average_distance


def corner_average_distance(first, second):
    if first is None or second is None:
        return 1000000.0
    total = 0.0
    for a, b in zip(first, second):
        total += distance(a, b)
    return total / 4.0


def blend_corners(old, new, old_weight=0.75):
    result = []
    for a, b in zip(old, new):
        result.append((
            int(round(a[0] * old_weight + b[0] * (1.0 - old_weight))),
            int(round(a[1] * old_weight + b[1] * (1.0 - old_weight))),
        ))
    return result


def draw_quad(img, quad, color, thickness=1):
    for i in range(4):
        a = quad[i]
        b = quad[(i + 1) % 4]
        img.draw_line((a[0], a[1], b[0], b[1]),
                      color=color, thickness=thickness)


def blob_corners(blob):
    try:
        return order_corners(blob.min_corners())
    except (AttributeError, ValueError, TypeError):
        return order_corners(blob.corners())


def find_board_blob(img, draw_debug=False):
    """Return the largest square-like low-density black connected region.

    Pipeline: grayscale → adaptive histogram equalization → binary
    threshold → blob detection. This normalizes local lighting so
    grid lines in highlight/shadow regions remain detectable.
    """
    # --- Lighting normalization ---
    gray = img.to_grayscale()
    gray = gray.histeq(adaptive=True)       # local contrast equalization
    gray = gray.gaussian(3)                 # suppress pixel noise before hard threshold
    gray.binary([GRAY_GRID_THRESHOLD])      # dark pixels → white blobs

    # Find white connected regions on the binary image (were dark grid lines)
    blobs = gray.find_blobs(
        [(128, 255)],                        # white pixels on binary image
        pixels_threshold=BLOB_PIXELS_THRESHOLD,
        area_threshold=BLOB_AREA_THRESHOLD,
        merge=True,
        margin=BLOB_MERGE_MARGIN,
    )
    image_area = gray.width() * gray.height()
    best_corners = None
    best_area = -1
    candidate_count = 0

    for index, blob in enumerate(blobs):
        try:
            corners = blob_corners(blob)
        except (ValueError, TypeError):
            continue
        density = blob.density()
        area = quad_area(corners)
        accepted = is_board_blob_geometry(corners, density,
                                          image_area, BOARD_LIMITS)
        if draw_debug:
            # Draw debug overlays on the ORIGINAL color image
            draw_quad(img, corners,
                      (255, 0, 255) if accepted else (0, 255, 255),
                      2 if accepted else 1)
            x = max(0, min(point[0] for point in corners))
            y = max(10, min(point[1] for point in corners) - 8)
            img.draw_string(x, y, "%d A%.0f D%d" %
                            (index + 1, area / image_area * 100,
                             int(density * 100)),
                            color=(255, 0, 255) if accepted else (0, 255, 255), scale=2)
        if accepted:
            candidate_count += 1
            if area > best_area:
                best_area = area
                best_corners = corners

    if draw_debug:
        img.draw_string(3, 3, "BLOBS %d CAND %d" %
                        (len(blobs), candidate_count),
                        color=(255, 255, 0), scale=2)
        if best_corners is not None:
            draw_quad(img, best_corners, (255, 255, 0), 3)
    return best_corners, len(blobs), candidate_count


def state_color(state):
    if state == BLACK:
        return (255, 0, 0)
    if state == WHITE:
        return (255, 255, 255)
    if state == EMPTY:
        return (0, 255, 0)
    return (255, 255, 0)


def sample_and_draw_cells(img, board_corners, rgb_filter, reference_rgb):
    states = []
    samples = []
    raw_values = []
    diffs = []
    counts = []
    cells = make_grid_cells(board_corners)
    for cell in cells:
        center, radius = incircle_geometry(cell, INCIRCLE_RADIUS_SCALE)
        rgb, count = circle_rgb_mean(img, center[0], center[1], radius,
                                     RGB_SAMPLE_STEP)
        if count < MIN_RGB_SAMPLES:
            rgb = None
        raw_values.append(rgb)
        counts.append(count)

    filtered_values = rgb_filter.update(raw_values)
    for index, cell in enumerate(cells):
        center, radius = incircle_geometry(cell, INCIRCLE_RADIUS_SCALE)
        raw_rgb = raw_values[index]
        filtered_rgb = filtered_values[index]

        # Classify by background subtraction
        ref_rgb = reference_rgb[index] if reference_rgb is not None else None
        state, diff = classify_by_diff(filtered_rgb, ref_rgb)
        states.append(state)
        diffs.append(diff)
        samples.append((raw_rgb, filtered_rgb, ref_rgb, counts[index], state, diff))

        draw_quad(img, cell, (0, 255, 255), 1)
        img.draw_circle((center[0], center[1], radius),
                        color=state_color(state), thickness=1)
        x = min(point[0] for point in cell) + 1
        y = min(point[1] for point in cell) + 1
        if filtered_rgb is None or ref_rgb is None:
            img.draw_string(x, y, "%d ---" % (index + 1),
                            color=(255, 255, 0), scale=2)
        else:
            img.draw_string(x, y, "%d D%d" % (index + 1, diff),
                            color=state_color(state), scale=2)
    return states, samples, diffs


def print_diff_stats(samples):
    diff_parts = []
    cur_parts = []
    ref_parts = []
    for index, sample in enumerate(samples):
        raw_rgb = sample[0]
        filtered_rgb = sample[1]
        ref_rgb = sample[2]
        diff = sample[5]
        diff_parts.append("%d:%d" % (index + 1, diff))
        if filtered_rgb is None:
            cur_parts.append("%d:---" % (index + 1))
        else:
            cur_parts.append("%d:R%d,G%d,B%d" %
                             (index + 1, filtered_rgb[0], filtered_rgb[1], filtered_rgb[2]))
        if ref_rgb is None:
            ref_parts.append("%d:---" % (index + 1))
        else:
            ref_parts.append("%d:R%d,G%d,B%d" %
                             (index + 1, ref_rgb[0], ref_rgb[1], ref_rgb[2]))
    print("差异 " + " | ".join(diff_parts))
    print("当前 " + " | ".join(cur_parts))
    print("参考 " + " | ".join(ref_parts))


def enable_auto_camera():
    try:
        sensor.set_auto_gain(True)
    except Exception:
        pass
    try:
        sensor.set_auto_whitebal(True)
    except Exception:
        pass
    try:
        sensor.set_auto_exposure(True)
    except Exception:
        pass


def freeze_camera():
    try:
        sensor.set_auto_gain(False)
    except Exception:
        pass
    try:
        sensor.set_auto_whitebal(False)
    except Exception:
        pass
    try:
        sensor.set_auto_exposure(False)
    except Exception:
        pass


sensor = None

try:
    # ---- K230 CanMV sensor init ----
    sensor = Sensor(width=WIDTH, height=HEIGHT, fps=30)
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    # K230 CanMV does not support skip_frames, use sleep instead
    time.sleep_ms(2000)

    # ---- K230 CanMV display & media ----
    Display.init(Display.ST7701, width=640, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()

    enable_auto_camera()

    clock = time.clock()
    debouncer = BoardDebouncer(BOARD_STABLE_FRAMES)
    rgb_filter = RGBMedianFilter(9, RGB_FILTER_WINDOW)
    locked_board = None
    candidate_board = None
    candidate_count = 0
    validation_misses = 0
    frame_count = 0
    last_printed_board = None
    settle_remaining = 0

    # ---- Background subtraction state ----
    reference_rgb = None         # 9-tuple of (r,g,b) reference for each cell
    ref_buffer = []              # multi-frame buffer during reference capture
    capturing_reference = False  # True while collecting reference frames
    all_empty_count = 0          # consecutive all-empty frames for auto re-ref

    print("K230 CanMV 背景减法井字棋启动")
    print("编码: 0=空, 1=白, 2=黑, -1=未知")

    while True:
        clock.tick()
        os.exitpoint()
        img = sensor.snapshot()
        frame_count += 1

        if locked_board is None:
            found, blob_count, accepted_count = find_board_blob(img, True)
            if found is None:
                candidate_board = None
                candidate_count = 0
            else:
                if corners_close(candidate_board, found, LOCK_CORNER_DISTANCE):
                    candidate_count += 1
                    candidate_board = blend_corners(candidate_board, found)
                else:
                    candidate_board = found
                    candidate_count = 1
                img.draw_string(3, 14, "LOCK %d/%d" %
                                (candidate_count, LOCK_STABLE_FRAMES),
                                color=(255, 255, 0), scale=2)
                if candidate_count >= LOCK_STABLE_FRAMES:
                    locked_board = candidate_board[:]
                    validation_misses = 0
                    debouncer.reset()
                    rgb_filter.reset()
                    last_printed_board = None
                    settle_remaining = EXPOSURE_SETTLE_FRAMES
                    print("棋盘已锁定", locked_board)
            Display.show_image(img)
            gc.collect()
            continue

        # Track the grid on every frame. Never sample RGB using stale corners.
        found, _, _ = find_board_blob(img, False)
        if found is None:
            validation_misses += 1
            draw_quad(img, locked_board, (255, 0, 0), 2)
            img.draw_string(3, 3, "FRAME MISS %d/%d" %
                            (validation_misses, UNLOCK_AFTER_MISSES),
                            color=(255, 0, 0), scale=2)
            if validation_misses >= UNLOCK_AFTER_MISSES:
                print("棋盘丢失: 返回斑点搜索")
                locked_board = None
                candidate_board = None
                candidate_count = 0
                reference_rgb = None
                capturing_reference = False
                ref_buffer = []
                all_empty_count = 0
                debouncer.reset()
                rgb_filter.reset()
                enable_auto_camera()
            Display.show_image(img)
            gc.collect()
            continue

        validation_misses = 0
        movement = corner_average_distance(locked_board, found)
        if movement <= 3.0:
            locked_board = blend_corners(locked_board, found, 0.92)
        elif movement <= 15.0:
            locked_board = blend_corners(locked_board, found, 0.65)
        elif movement <= MAX_TRACK_JUMP:
            locked_board = found[:]
            rgb_filter.reset()
            debouncer.reset()
        else:
            # A valid grid that moved far must be followed immediately, not sampled
            # with stale corners. Geometry/density filtering already rejected solids.
            locked_board = found[:]
            rgb_filter.reset()
            debouncer.reset()

        # Let exposure/gain/white balance adapt to the locked white board first.
        if settle_remaining > 0:
            draw_quad(img, locked_board, (255, 255, 0), 3)
            img.draw_string(3, 3, "CAMERA SETTLE %d" % settle_remaining,
                            color=(255, 255, 0), scale=2)
            settle_remaining -= 1
            if settle_remaining == 0:
                freeze_camera()
                rgb_filter.reset()
                debouncer.reset()
                # Start reference capture on next frame
                capturing_reference = True
                ref_buffer = []
                print("相机已冻结: 正在采集参考...")
            Display.show_image(img)
            gc.collect()
            continue

        # ---- Reference capture phase ----
        if capturing_reference:
            frame_refs = capture_reference_rgb(img, locked_board)
            ref_buffer.append(frame_refs)
            draw_quad(img, locked_board, (255, 255, 0), 3)
            img.draw_string(3, 3, "REF CAP %d/%d" %
                            (len(ref_buffer), REFERENCE_FRAMES),
                            color=(255, 255, 0), scale=2)
            if len(ref_buffer) >= REFERENCE_FRAMES:
                # Take per-cell median across frames as reference
                reference_rgb = []
                for cell_idx in range(9):
                    cell_rgbs = [buf[cell_idx] for buf in ref_buffer
                                 if buf[cell_idx] is not None]
                    if not cell_rgbs:
                        reference_rgb.append(None)
                        continue
                    rs = sorted(r[0] for r in cell_rgbs)
                    gs = sorted(r[1] for r in cell_rgbs)
                    bs = sorted(r[2] for r in cell_rgbs)
                    mid = len(cell_rgbs) // 2
                    reference_rgb.append((rs[mid], gs[mid], bs[mid]))
                capturing_reference = False
                ref_buffer = []
                all_empty_count = 0
                print("参考已采集")
                ref_parts = []
                for idx, rgb in enumerate(reference_rgb):
                    if rgb is None:
                        ref_parts.append("%d:---" % (idx + 1))
                    else:
                        ref_parts.append("%d:R%d,G%d,B%d" %
                                         (idx + 1, rgb[0], rgb[1], rgb[2]))
                print("参考RGB " + " | ".join(ref_parts))
            Display.show_image(img)
            gc.collect()
            continue

        if reference_rgb is None:
            # Should not normally reach here, but guard
            draw_quad(img, locked_board, (255, 255, 0), 3)
            img.draw_string(3, 3, "WAIT REF...",
                            color=(255, 255, 0), scale=2)
            Display.show_image(img)
            gc.collect()
            continue

        raw_board, samples, diffs = sample_and_draw_cells(
            img, locked_board, rgb_filter, reference_rgb)
        stable_board = debouncer.update(raw_board)
        draw_quad(img, locked_board, (0, 255, 0), 3)
        status_line = "GRID OK %d" % (clock.fps())
        img.draw_string(3, 3, status_line,
                        color=(0, 255, 0), scale=2)

        # ---- Auto re-reference: slow EMA when all 9 cells are EMPTY ----
        if all(s == EMPTY for s in raw_board) and all(d < DIFF_THRESHOLD
                                                       for d in diffs):
            all_empty_count += 1
            if all_empty_count >= AUTO_REF_FRAMES and REF_EMA_ALPHA > 0:
                # EMA update reference toward current filtered values
                for idx in range(9):
                    cur = samples[idx][1]  # filtered_rgb
                    ref = reference_rgb[idx]
                    if cur is not None and ref is not None:
                        reference_rgb[idx] = (
                            int(round(ref[0] * (1 - REF_EMA_ALPHA) +
                                      cur[0] * REF_EMA_ALPHA)),
                            int(round(ref[1] * (1 - REF_EMA_ALPHA) +
                                      cur[1] * REF_EMA_ALPHA)),
                            int(round(ref[2] * (1 - REF_EMA_ALPHA) +
                                      cur[2] * REF_EMA_ALPHA)),
                        )
                all_empty_count = 0
                print("自动参考已更新")
        else:
            all_empty_count = 0

        if frame_count % PRINT_EVERY == 0:
            print_diff_stats(samples)
            print("原始棋盘", raw_board)

        if stable_board is not None and stable_board != last_printed_board:
            print("棋盘", stable_board)
            last_printed_board = stable_board[:]

        Display.show_image(img)
        gc.collect()

except KeyboardInterrupt as e:
    print("用户停止")
except BaseException as e:
    print("异常 '%s'" % e)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
