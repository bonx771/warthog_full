#!/usr/bin/env python3

import math
import statistics


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM_K0 = 0.9996


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def latlon_to_utm(latitude, longitude, forced_zone=None):
    """Convert WGS84 latitude/longitude to UTM without an external library."""
    latitude = float(latitude)
    longitude = float(longitude)
    if not (-80.0 <= latitude <= 84.0):
        raise ValueError("UTM supports latitude from -80 to 84 degrees")
    if not (-180.0 <= longitude <= 180.0):
        raise ValueError("longitude must be in [-180, 180]")

    if forced_zone is None:
        zone = int((longitude + 180.0) / 6.0) + 1
        zone = min(60, max(1, zone))
    else:
        zone = int(forced_zone)
        if zone < 1 or zone > 60:
            raise ValueError("UTM zone must be in [1, 60]")

    lat_rad = math.radians(latitude)
    lon_rad = math.radians(longitude)
    lon_origin = math.radians((zone - 1) * 6 - 180 + 3)

    e_sq = WGS84_F * (2.0 - WGS84_F)
    e_prime_sq = e_sq / (1.0 - e_sq)
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    tan_lat = math.tan(lat_rad)
    n = WGS84_A / math.sqrt(1.0 - e_sq * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = e_prime_sq * cos_lat * cos_lat
    aa = cos_lat * (lon_rad - lon_origin)

    m = WGS84_A * (
        (1.0 - e_sq / 4.0 - 3.0 * e_sq ** 2 / 64.0 - 5.0 * e_sq ** 3 / 256.0) * lat_rad
        - (3.0 * e_sq / 8.0 + 3.0 * e_sq ** 2 / 32.0 + 45.0 * e_sq ** 3 / 1024.0)
        * math.sin(2.0 * lat_rad)
        + (15.0 * e_sq ** 2 / 256.0 + 45.0 * e_sq ** 3 / 1024.0)
        * math.sin(4.0 * lat_rad)
        - (35.0 * e_sq ** 3 / 3072.0) * math.sin(6.0 * lat_rad)
    )

    easting = UTM_K0 * n * (
        aa
        + (1.0 - t + c) * aa ** 3 / 6.0
        + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * e_prime_sq)
        * aa ** 5
        / 120.0
    ) + 500000.0

    northing = UTM_K0 * (
        m
        + n
        * tan_lat
        * (
            aa * aa / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c * c) * aa ** 4 / 24.0
            + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * e_prime_sq)
            * aa ** 6
            / 720.0
        )
    )

    hemisphere = "N"
    if latitude < 0.0:
        northing += 10000000.0
        hemisphere = "S"
    return easting, northing, zone, hemisphere


def rotate_xy(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def antenna_to_base(easting, northing, base_yaw, antenna_x, antenna_y):
    offset_e, offset_n = rotate_xy(antenna_x, antenna_y, base_yaw)
    return easting - offset_e, northing - offset_n


def sensor_to_base(map_x, map_y, base_yaw, sensor_x, sensor_y):
    offset_x, offset_y = rotate_xy(sensor_x, sensor_y, base_yaw)
    return map_x - offset_x, map_y - offset_y


def transform_xy(x, y, origin_x, origin_y, target_x, target_y, yaw):
    dx = x - origin_x
    dy = y - origin_y
    rx, ry = rotate_xy(dx, dy, yaw)
    return target_x + rx, target_y + ry


def _fit_once(samples):
    if len(samples) < 2:
        raise ValueError("at least two samples are required")

    source_x = statistics.fmean(sample[0] for sample in samples)
    source_y = statistics.fmean(sample[1] for sample in samples)
    target_x = statistics.fmean(sample[2] for sample in samples)
    target_y = statistics.fmean(sample[3] for sample in samples)

    cross = 0.0
    dot = 0.0
    for sx, sy, tx, ty, *_unused in samples:
        ux = sx - source_x
        uy = sy - source_y
        vx = tx - target_x
        vy = ty - target_y
        cross += ux * vy - uy * vx
        dot += ux * vx + uy * vy

    if abs(cross) + abs(dot) < 1e-12:
        raise ValueError("samples have insufficient positional spread")

    yaw = math.atan2(cross, dot)
    residuals = []
    for sx, sy, tx, ty, *_unused in samples:
        px, py = transform_xy(sx, sy, source_x, source_y, target_x, target_y, yaw)
        residuals.append(math.hypot(px - tx, py - ty))

    return {
        "utm_origin_easting": source_x,
        "utm_origin_northing": source_y,
        "map_origin_x": target_x,
        "map_origin_y": target_y,
        "utm_to_map_yaw": yaw,
        "residuals": residuals,
    }


def fit_se2_robust(samples, absolute_outlier_m=0.75, max_iterations=4):
    """Fit UTM->map SE(2), rejecting large translation residuals."""
    inliers = list(samples)
    if len(inliers) < 2:
        raise ValueError("at least two samples are required")

    for _ in range(max(1, int(max_iterations))):
        fit = _fit_once(inliers)
        residuals = fit["residuals"]
        median = statistics.median(residuals)
        mad = statistics.median(abs(value - median) for value in residuals)
        robust_limit = median + 3.5 * max(1.4826 * mad, 0.02)
        limit = max(float(absolute_outlier_m), robust_limit)
        filtered = [sample for sample, error in zip(inliers, residuals) if error <= limit]
        if len(filtered) < 2 or len(filtered) == len(inliers):
            break
        inliers = filtered

    fit = _fit_once(inliers)
    residuals = fit.pop("residuals")
    fit["inliers"] = inliers
    fit["rms_error_m"] = math.sqrt(statistics.fmean(error * error for error in residuals))
    fit["max_error_m"] = max(residuals)
    fit["median_error_m"] = statistics.median(residuals)
    return fit


def trajectory_span(samples):
    if not samples:
        return 0.0
    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def circular_mean(values):
    values = list(values)
    if not values:
        raise ValueError("no angles supplied")
    sine = statistics.fmean(math.sin(value) for value in values)
    cosine = statistics.fmean(math.cos(value) for value in values)
    return math.atan2(sine, cosine)


def circular_rms(values, center):
    values = list(values)
    if not values:
        return 0.0
    errors = [normalize_angle(value - center) for value in values]
    return math.sqrt(statistics.fmean(error * error for error in errors))
