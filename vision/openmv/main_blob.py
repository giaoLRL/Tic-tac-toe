"""OpenMV4P tic-tac-toe recognition via the largest black grid blob.

This mirrors sanqi.py's largest-contour strategy without OpenCV:
black threshold -> connected blobs -> largest rotated rectangle -> 3x3 grid.
Save vision_core_rgb.py to the OpenMV device before running this file.
"""

import sensor
import time

from vision_core_rgb import (
    BLACK,
    EMPTY,
    UNKNOWN,
    WHITE,
    BoardDebouncer,
    RGBMedianFilter,
    circle_rgb_mean,
    classify_rgb,
    distance,
    incircle_geometry,
    is_board_blob_geometry,
    make_grid_cells,
    order_corners,
    quad_area,
)


# ======================== User calibration area ========================
# LAB threshold for black printed grid lines on a white board.
BLACK_GRID_THRESHOLD = (0, 50, -25, 25, -25, 25)

BLOB_PIXELS_THRESHOLD = 80
BLOB_AREA_THRESHOLD = 800
BLOB_MERGE_MARGIN = 4

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
LOCK_CORNER_DISTANCE = 12.0
MAX_TRACK_JUMP = 55.0
UNLOCK_AFTER_MISSES = 6
EXPOSURE_SETTLE_FRAMES = 20

INCIRCLE_RADIUS_SCALE = 0.28
RGB_SAMPLE_STEP = 2
MIN_RGB_SAMPLES = 12
BOARD_STABLE_FRAMES = 5
RGB_FILTER_WINDOW = 5

RGB_THRESHOLDS = {
    "black_g": (0, 55),
    "black_b": (0, 30),
    "white_r": (225, 255),
    "white_g": (210, 255),
}

PRINT_EVERY = 20
# ======================================================================


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
    """Return the largest square-like low-density black connected region."""
    blobs = img.find_blobs(
        [BLACK_GRID_THRESHOLD],
        pixels_threshold=BLOB_PIXELS_THRESHOLD,
        area_threshold=BLOB_AREA_THRESHOLD,
        merge=True,
        margin=BLOB_MERGE_MARGIN,
    )
    image_area = img.width() * img.height()
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
            draw_quad(img, corners,
                      (255, 0, 255) if accepted else (0, 255, 255),
                      2 if accepted else 1)
            x = max(0, min(point[0] for point in corners))
            y = max(10, min(point[1] for point in corners) - 8)
            img.draw_string(x, y, "%d A%.0f D%d" %
                            (index + 1, area / image_area * 100,
                             int(density * 100)),
                            color=(255, 0, 255) if accepted else (0, 255, 255),
                            scale=1)
        if accepted:
            candidate_count += 1
            if area > best_area:
                best_area = area
                best_corners = corners

    if draw_debug:
        img.draw_string(3, 3, "BLOBS %d CAND %d" %
                        (len(blobs), candidate_count),
                        color=(255, 255, 0), scale=1)
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


def sample_and_draw_cells(img, board_corners, rgb_filter):
    states = []
    samples = []
    raw_values = []
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
        state = classify_rgb(filtered_rgb, RGB_THRESHOLDS)
        states.append(state)
        samples.append((raw_rgb, filtered_rgb, counts[index], state))

        draw_quad(img, cell, (0, 255, 255), 1)
        img.draw_circle((center[0], center[1], radius),
                        color=state_color(state), thickness=1)
        x = min(point[0] for point in cell) + 1
        y = min(point[1] for point in cell) + 1
        if filtered_rgb is None:
            img.draw_string(x, y, "%d ---" % (index + 1),
                            color=(255, 255, 0), scale=1)
        else:
            img.draw_string(x, y, "%d R%d" % (index + 1, filtered_rgb[0]),
                            color=(255, 0, 0), scale=1)
            img.draw_string(x, y + 9, "G%d" % filtered_rgb[1],
                            color=(0, 255, 0), scale=1)
            img.draw_string(x, y + 18, "B%d" % filtered_rgb[2],
                            color=(0, 128, 255), scale=1)
    return states, samples


def print_rgb_stats(samples):
    raw_parts = []
    filtered_parts = []
    for index, sample in enumerate(samples):
        raw_rgb = sample[0]
        filtered_rgb = sample[1]
        if raw_rgb is None:
            raw_parts.append("%d:---" % (index + 1))
        else:
            raw_parts.append("%d:R%d,G%d,B%d" %
                             (index + 1, raw_rgb[0], raw_rgb[1], raw_rgb[2]))
        if filtered_rgb is None:
            filtered_parts.append("%d:---" % (index + 1))
        else:
            filtered_parts.append("%d:R%d,G%d,B%d" %
                                  (index + 1, filtered_rgb[0],
                                   filtered_rgb[1], filtered_rgb[2]))
    print("RAW_RGB " + " | ".join(raw_parts))
    print("FILTERED_RGB " + " | ".join(filtered_parts))


def enable_auto_camera():
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)
    try:
        sensor.set_auto_exposure(True)
    except Exception:
        pass


def freeze_camera():
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    try:
        sensor.set_auto_exposure(False)
    except Exception:
        pass


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
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

print("OpenMV largest-grid-blob tic-tac-toe started")
print("Encoding: 0=EMPTY, 1=WHITE, 2=BLACK, -1=UNKNOWN")

while True:
    clock.tick()
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
                            color=(255, 255, 0), scale=1)
            if candidate_count >= LOCK_STABLE_FRAMES:
                locked_board = candidate_board[:]
                validation_misses = 0
                debouncer.reset()
                rgb_filter.reset()
                last_printed_board = None
                settle_remaining = EXPOSURE_SETTLE_FRAMES
                print("BOARD_LOCKED", locked_board)
        continue

    # Track the grid on every frame. Never sample RGB using stale corners.
    found, _, _ = find_board_blob(img, False)
    if found is None:
        validation_misses += 1
        draw_quad(img, locked_board, (255, 0, 0), 2)
        img.draw_string(3, 3, "FRAME MISS %d/%d" %
                        (validation_misses, UNLOCK_AFTER_MISSES),
                        color=(255, 0, 0), scale=1)
        if validation_misses >= UNLOCK_AFTER_MISSES:
            print("BOARD_LOST: returning to blob search")
            locked_board = None
            candidate_board = None
            candidate_count = 0
            debouncer.reset()
            rgb_filter.reset()
            enable_auto_camera()
        continue

    validation_misses = 0
    movement = corner_average_distance(locked_board, found)
    if movement <= 3.0:
        locked_board = blend_corners(locked_board, found, 0.70)
    elif movement <= 15.0:
        locked_board = blend_corners(locked_board, found, 0.30)
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
                        color=(255, 255, 0), scale=1)
        settle_remaining -= 1
        if settle_remaining == 0:
            freeze_camera()
            rgb_filter.reset()
            debouncer.reset()
            print("CAMERA_FROZEN: starting RGB sampling")
        continue

    raw_board, samples = sample_and_draw_cells(img, locked_board, rgb_filter)
    stable_board = debouncer.update(raw_board)
    draw_quad(img, locked_board, (0, 255, 0), 3)
    img.draw_string(3, 3, "GRID LOCKED %.1f" % clock.fps(),
                    color=(0, 255, 0), scale=1)

    if frame_count % PRINT_EVERY == 0:
        print_rgb_stats(samples)
        print("RAW_BOARD", raw_board)

    if stable_board is not None and stable_board != last_printed_board:
        print("BOARD", stable_board)
        last_printed_board = stable_board[:]
