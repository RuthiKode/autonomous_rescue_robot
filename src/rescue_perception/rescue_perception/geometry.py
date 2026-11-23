"""Depth and camera geometry for the survivor localizer. Pure numpy, no ROS."""
import math

import numpy as np


def median_depth(depth, centre_x, centre_y, width, height, central_fraction, min_range,
                 max_range, min_pixels=10):
    """Median depth over the central part of a bounding box, or None.

    depth              2D float array (metres); NaN, inf and out-of-range values are ignored
    centre_x, centre_y bounding-box centre in pixels
    width, height      bounding-box size in pixels
    central_fraction   share of the box (per side) that is used; the median ignores the
                       background pixels that fall inside a person-shaped box
    """
    h, w = depth.shape
    half_w, half_h = 0.5 * width * central_fraction, 0.5 * height * central_fraction
    c0, c1 = max(0, int(round(centre_x - half_w))), min(w, int(round(centre_x + half_w)) + 1)
    r0, r1 = max(0, int(round(centre_y - half_h))), min(h, int(round(centre_y + half_h)) + 1)
    if c0 >= c1 or r0 >= r1:
        return None
    patch = depth[r0:r1, c0:c1]
    valid = patch[np.isfinite(patch) & (patch > min_range) & (patch < max_range)]
    if valid.size < min_pixels:
        return None
    return float(np.median(valid))


def backproject(u, v, z, fx, fy, cx, cy):
    """Pixel (u, v) at depth z along the optical axis to a point in the optical frame.

    Optical convention: x right, y down, z forward.
    """
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def quat_rotate(q, v):
    """Rotate vector v by the unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2w (u x v) + 2 u x (u x v), with u = (x, y, z)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def transform_point(translation, rotation, point):
    """Apply a transform (translation, quaternion xyzw) to a point."""
    rx, ry, rz = quat_rotate(rotation, point)
    return (rx + translation[0], ry + translation[1], rz + translation[2])


def inverse_transform_point(translation, rotation, point):
    """Express a point given in the parent frame in the child frame (inverse of transform_point)."""
    x, y, z, w = rotation
    return quat_rotate((-x, -y, -z, w), (point[0] - translation[0], point[1] - translation[1],
                                         point[2] - translation[2]))


def point_visible(point_cam, depth, fx, fy, cx, cy, min_range, max_range, margin=20,
                  occlusion_tol=0.6, window=2):
    """Could the camera see a point given in the optical frame (x right, y down, z forward)?

    True when the point is within the depth range, projects inside the image (minus `margin`
    pixels) and no surface in the depth image is more than `occlusion_tol` metres nearer than
    the point along that pixel. Invalid depth (NaN, inf) counts as "nothing in the way".
    """
    x, y, z = point_cam
    if not (min_range < z < max_range):
        return False
    h, w = depth.shape
    u, v = fx * x / z + cx, fy * y / z + cy
    if not (margin <= u < w - margin and margin <= v < h - margin):
        return False
    patch = depth[max(0, int(v) - window):int(v) + window + 1,
                  max(0, int(u) - window):int(u) + window + 1]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return True
    return float(np.median(valid)) >= z - occlusion_tol


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
