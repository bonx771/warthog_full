import math
from collections import defaultdict


def rotate_vector_by_quaternion(vector, quaternion):
    """Rotate a 3-D vector by a quaternion given as (x, y, z, w)."""
    vx, vy, vz = vector
    qx, qy, qz, qw = quaternion

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("invalid quaternion")
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    # q * v * q^-1, expanded to avoid an external geometry dependency.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def ground_anchor_from_pose(position, quaternion, lidar_height):
    """Return the ground point directly below a LiDAR pose."""
    offset = rotate_vector_by_quaternion((0.0, 0.0, -lidar_height), quaternion)
    return (
        position[0] + offset[0],
        position[1] + offset[1],
        position[2] + offset[2],
    )


def thin_ordered_anchors(anchors, minimum_spacing):
    """Remove consecutive stationary samples while preserving the path ends."""
    if not anchors:
        return []
    if minimum_spacing <= 0.0:
        return list(anchors)

    kept = [anchors[0]]
    pending = None
    for anchor in anchors[1:]:
        previous = kept[-1]
        if math.hypot(anchor[0] - previous[0], anchor[1] - previous[1]) >= minimum_spacing:
            kept.append(anchor)
            pending = None
        else:
            pending = anchor
    if pending is not None:
        previous = kept[-1]
        distance = math.hypot(pending[0] - previous[0], pending[1] - previous[1])
        if distance < 1.0e-6:
            # Keep the newest optimized pose for a truly stationary end segment.
            kept[-1] = pending
        else:
            # Preserve the final path endpoint even when the last segment is short.
            kept.append(pending)
    return kept


def median_smooth_anchor_heights(anchors, window_size):
    """Suppress isolated A-LOAM Z spikes without changing anchor X/Y."""
    if window_size <= 1 or len(anchors) <= 2:
        return list(anchors)
    window_size = max(1, int(window_size))
    radius = window_size // 2
    smoothed = []
    for index, anchor in enumerate(anchors):
        start = max(0, index - radius)
        stop = min(len(anchors), index + radius + 1)
        heights = sorted(item[2] for item in anchors[start:stop])
        middle = len(heights) // 2
        if len(heights) % 2:
            height = heights[middle]
        else:
            height = 0.5 * (heights[middle - 1] + heights[middle])
        smoothed.append((anchor[0], anchor[1], height))
    return smoothed


class PolylineCorridor:
    """Test whether XY points lie inside a constant-width path corridor."""

    def __init__(self, points, half_width, cell_size=None):
        if half_width <= 0.0 or not math.isfinite(half_width):
            raise ValueError("half_width must be positive and finite")

        valid_points = []
        for point in points:
            if len(point) < 2:
                continue
            x = float(point[0])
            y = float(point[1])
            if math.isfinite(x) and math.isfinite(y):
                valid_points.append((x, y))
        if not valid_points:
            raise ValueError("corridor requires at least one finite XY point")

        self.half_width = float(half_width)
        self.half_width_sq = self.half_width * self.half_width
        if cell_size is None:
            cell_size = max(1.0, self.half_width)
        if cell_size <= 0.0 or not math.isfinite(cell_size):
            raise ValueError("cell_size must be positive and finite")
        self.cell_size = float(cell_size)
        self.cells = defaultdict(list)
        self.segments = []

        if len(valid_points) == 1:
            self._add_segment(valid_points[0], valid_points[0])
        else:
            for start, end in zip(valid_points, valid_points[1:]):
                self._add_segment(start, end)

    def _cell(self, x, y):
        return (
            int(math.floor(x / self.cell_size)),
            int(math.floor(y / self.cell_size)),
        )

    def _add_segment(self, start, end):
        segment_index = len(self.segments)
        self.segments.append((start, end))
        min_cell = self._cell(
            min(start[0], end[0]) - self.half_width,
            min(start[1], end[1]) - self.half_width,
        )
        max_cell = self._cell(
            max(start[0], end[0]) + self.half_width,
            max(start[1], end[1]) + self.half_width,
        )
        for cell_x in range(min_cell[0], max_cell[0] + 1):
            for cell_y in range(min_cell[1], max_cell[1] + 1):
                self.cells[(cell_x, cell_y)].append(segment_index)

    @staticmethod
    def _distance_sq_to_segment(x, y, start, end):
        segment_x = end[0] - start[0]
        segment_y = end[1] - start[1]
        length_sq = segment_x * segment_x + segment_y * segment_y
        if length_sq < 1.0e-12:
            nearest_x = start[0]
            nearest_y = start[1]
        else:
            projection = (
                (x - start[0]) * segment_x + (y - start[1]) * segment_y
            ) / length_sq
            projection = max(0.0, min(1.0, projection))
            nearest_x = start[0] + projection * segment_x
            nearest_y = start[1] + projection * segment_y
        delta_x = x - nearest_x
        delta_y = y - nearest_y
        return delta_x * delta_x + delta_y * delta_y

    def contains(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        for segment_index in self.cells.get(self._cell(x, y), ()):
            start, end = self.segments[segment_index]
            if self._distance_sq_to_segment(x, y, start, end) <= self.half_width_sq:
                return True
        return False


class GroundSurface:
    """Spatially indexed inverse-distance ground-height interpolation."""

    def __init__(self, anchors, cell_size=2.0, neighbor_count=8, max_distance=30.0):
        if not anchors:
            raise ValueError("at least one ground anchor is required")
        if cell_size <= 0.0:
            raise ValueError("cell_size must be positive")
        if neighbor_count < 1:
            raise ValueError("neighbor_count must be at least one")
        if max_distance <= 0.0:
            raise ValueError("max_distance must be positive")

        self.cell_size = float(cell_size)
        self.neighbor_count = int(neighbor_count)
        self.max_distance = float(max_distance)
        self.max_distance_sq = self.max_distance * self.max_distance
        self.cells = defaultdict(list)
        for anchor in anchors:
            if len(anchor) != 3 or not all(math.isfinite(value) for value in anchor):
                continue
            self.cells[self._cell(anchor[0], anchor[1])].append(tuple(anchor))
        if not self.cells:
            raise ValueError("no finite ground anchors were provided")

    def _cell(self, x, y):
        return (
            int(math.floor(x / self.cell_size)),
            int(math.floor(y / self.cell_size)),
        )

    @staticmethod
    def _ring_cells(center_x, center_y, radius):
        if radius == 0:
            yield center_x, center_y
            return
        low_x = center_x - radius
        high_x = center_x + radius
        low_y = center_y - radius
        high_y = center_y + radius
        for cell_x in range(low_x, high_x + 1):
            yield cell_x, low_y
            yield cell_x, high_y
        for cell_y in range(low_y + 1, high_y):
            yield low_x, cell_y
            yield high_x, cell_y

    def height(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None

        center_x, center_y = self._cell(x, y)
        candidates = []
        max_ring = int(math.ceil(self.max_distance / self.cell_size)) + 1
        for radius in range(max_ring + 1):
            for cell in self._ring_cells(center_x, center_y, radius):
                for anchor in self.cells.get(cell, ()):
                    dx = anchor[0] - x
                    dy = anchor[1] - y
                    distance_sq = dx * dx + dy * dy
                    if distance_sq <= self.max_distance_sq:
                        candidates.append((distance_sq, anchor[2]))

            if len(candidates) >= self.neighbor_count:
                nearest = sorted(candidates, key=lambda item: item[0])[: self.neighbor_count]
                kth_distance = math.sqrt(nearest[-1][0])
                left = (center_x - radius) * self.cell_size
                right = (center_x + radius + 1) * self.cell_size
                bottom = (center_y - radius) * self.cell_size
                top = (center_y + radius + 1) * self.cell_size
                distance_to_unsearched = min(x - left, right - x, y - bottom, top - y)
                if kth_distance <= distance_to_unsearched:
                    candidates = nearest
                    break

        if not candidates:
            return None
        nearest = sorted(candidates, key=lambda item: item[0])[: self.neighbor_count]
        if nearest[0][0] < 1.0e-8:
            return nearest[0][1]

        # A 0.25 m softening term prevents one noisy pose from dominating.
        weighted_height = 0.0
        total_weight = 0.0
        for distance_sq, height in nearest:
            weight = 1.0 / (distance_sq + 0.25 * 0.25)
            weighted_height += weight * height
            total_weight += weight
        return weighted_height / total_weight


def disk_cell_offsets(radius_m, resolution):
    if radius_m <= 0.0:
        return [(0, 0)]
    radius_cells = int(math.ceil(radius_m / resolution))
    limit_sq = (radius_m / resolution + 0.5) ** 2
    offsets = []
    for row_offset in range(-radius_cells, radius_cells + 1):
        for col_offset in range(-radius_cells, radius_cells + 1):
            if row_offset * row_offset + col_offset * col_offset <= limit_sq:
                offsets.append((row_offset, col_offset))
    return offsets


def dilate_cells(cells, width, height, radius_m, resolution):
    offsets = disk_cell_offsets(radius_m, resolution)
    if len(offsets) == 1:
        return set(cells)
    result = set()
    for row, col in cells:
        for row_offset, col_offset in offsets:
            expanded_row = row + row_offset
            expanded_col = col + col_offset
            if 0 <= expanded_row < height and 0 <= expanded_col < width:
                result.add((expanded_row, expanded_col))
    return result


def clear_corridor_cells(cells, x_min, y_min, resolution, corridor):
    """Remove occupied cells whose centers lie inside a path corridor."""
    result = set()
    for row, col in cells:
        x = x_min + (col + 0.5) * resolution
        y = y_min + (row + 0.5) * resolution
        if not corridor.contains(x, y):
            result.add((row, col))
    return result
