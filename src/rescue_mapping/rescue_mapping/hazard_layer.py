"""Hazard layer: mark cells in passages narrower than the robot's width plus a margin.

Subscribes to /map (nav_msgs/OccupancyGrid) and publishes /hazard_map on the same grid:

    100  known-free cell inside a narrow passage (the hazard)
      0  any other known-free cell
     -1  occupied or unknown

Method (a medial-axis "neck" search; see config/hazard.yaml for the definition):

1. D = distance from every cell to the nearest obstacle. Unknown cells are not obstacles,
   so a half-explored corridor is not called narrow because its sides are unknown.
2. The ridge is the set of local maxima of D: the medial axis. Its value D is the radius of
   the widest disk that fits at that point.
3. Ridge cells with D below r = (robot width + margin) / 2 are narrow, the rest wide.
   Narrow ridge cells form connected pieces.
4. A narrow piece is a passage when it touches the wide ridge in two or more separate
   places: it links two wide regions through a stretch too narrow for the robot. A piece
   that touches the wide ridge once is not (corner wedges in a room, dead-end alcoves).
   A piece that touches it nowhere is a region that is narrow throughout and also counts.
5. The hazard is the union of the maximal disks (radius D) along those pieces, on
   known-free cells, with regions smaller than min_area_m2 dropped.

A plain morphological opening was tried first and rejected: a disk of the robot's width
reaches into a short gap from both sides and covers it, so thin 0.7 m gaps were missed.
"""
import numpy as np
from scipy import ndimage

OCCUPIED, FREE, UNKNOWN = 100, 0, -1
EIGHT = np.ones((3, 3), dtype=bool)


def disk(radius_cells):
    k = int(np.ceil(radius_cells))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    return xx ** 2 + yy ** 2 <= radius_cells ** 2


def hazard_cells(grid, resolution, robot_width_m, margin_m, min_area_m2, occupied_threshold=65,
                 min_width_m=0.28, contact_cells=3):
    """Return a boolean array, True on hazard cells.

    grid           2D int array of occupancy values (-1 unknown, 0..100), any orientation
    min_width_m    ridge points narrower than this are ignored: the robot cannot enter them
                   at all, and the costmap already treats them as blocked
    contact_cells  how far (cells) a narrow piece may be from the wide ridge and still count
                   as touching it
    """
    occupied = grid >= occupied_threshold
    known_free = (grid >= 0) & ~occupied
    r = 0.5 * (robot_width_m + margin_m) / resolution              # cells
    dist = ndimage.distance_transform_edt(~occupied)
    ridge = (dist >= ndimage.maximum_filter(dist, footprint=EIGHT)) \
        & (dist >= 0.5 * min_width_m / resolution)
    wide = ridge & (dist >= r)
    narrow = ridge & (dist < r)

    pieces, count = ndimage.label(narrow, structure=EIGHT)
    hazard = np.zeros(grid.shape, dtype=bool)
    if count == 0:
        return hazard
    pad = int(np.ceil(r)) + contact_cells + 1
    for idx, sl in enumerate(ndimage.find_objects(pieces), start=1):
        # Work on a window around the piece.
        r0, r1 = max(0, sl[0].start - pad), min(grid.shape[0], sl[0].stop + pad)
        c0, c1 = max(0, sl[1].start - pad), min(grid.shape[1], sl[1].stop + pad)
        piece = pieces[r0:r1, c0:c1] == idx
        near = ndimage.binary_dilation(piece, structure=EIGHT, iterations=contact_cells)
        _, contacts = ndimage.label(near & wide[r0:r1, c0:c1], structure=EIGHT)
        if contacts == 1:
            continue                       # corner wedge or dead end
        window = hazard[r0:r1, c0:c1]
        for y, x in zip(*np.nonzero(piece)):
            rad = dist[r0 + y, c0 + x] + 0.5
            k = int(np.ceil(rad))
            ys, xs = slice(max(0, y - k), y + k + 1), slice(max(0, x - k), x + k + 1)
            dy0, dx0 = ys.start - (y - k), xs.start - (x - k)
            stamp = disk(rad)[dy0:dy0 + window[ys, xs].shape[0], dx0:dx0 + window[ys, xs].shape[1]]
            window[ys, xs] |= stamp
    hazard &= known_free

    # Drop small regions (speckle in the map).
    labels, n = ndimage.label(hazard, structure=EIGHT)
    if n:
        sizes = ndimage.sum(hazard, labels, index=np.arange(1, n + 1))
        keep = np.zeros(n + 1, dtype=bool)
        keep[1:] = sizes * resolution ** 2 >= min_area_m2
        hazard = keep[labels]
    return hazard


def hazard_grid(grid, resolution, **kwargs):
    """The published values: 100 hazard, 0 other known-free, -1 everything else."""
    haz = hazard_cells(grid, resolution, **kwargs)
    out = np.full(grid.shape, UNKNOWN, dtype=np.int8)
    out[(grid >= 0) & (grid < kwargs.get('occupied_threshold', 65))] = FREE
    out[haz] = OCCUPIED
    return out
