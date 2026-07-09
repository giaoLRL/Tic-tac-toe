"""MicroPython-compatible RGB incircle helpers for OpenMV tic-tac-toe."""

import math


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


def _inside(value, limits):
    return limits[0] <= value <= limits[1]


def classify_rgb(rgb, thresholds):
    """Classify black by G/B, white by R/G, and everything else as empty."""
    if rgb is None:
        return UNKNOWN
    red, green, blue = rgb
    if (_inside(green, thresholds["black_g"]) and
            _inside(blue, thresholds["black_b"])):
        return BLACK
    if (_inside(red, thresholds["white_r"]) and
            _inside(green, thresholds["white_g"])):
        return WHITE
    return EMPTY


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
