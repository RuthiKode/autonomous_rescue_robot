"""Ground-truth occupancy grids and the sensor covariance file."""
import os
import re
import struct
import zlib

import numpy as np
import pytest
import yaml

from rescue_simulation import generate_worlds as gw
from rescue_simulation.covariance_stamper import imu_covariances, odom_covariances

PKG = os.path.join(os.path.dirname(__file__), '..')
CFG = gw.load_config(os.path.join(PKG, 'config', 'world_generator.yaml'))
WORLDS = list(CFG['worlds'])
with open(os.path.join(PKG, '..', 'rescue_description', 'urdf', 'robot_params.xacro')) as _f:
    XACRO = _f.read()


def _xacro_value(name):
    return float(re.search(rf'name="{name}" value="([^"]+)"', XACRO).group(1))


def _builder(name):
    return gw.build(name, dict(CFG['worlds'][name]), CFG['common'], CFG['seed'])


def _read_png_gray(path):
    with open(path, 'rb') as f:
        data = f.read()
    width, height = struct.unpack('>II', data[16:24])
    pos, idat = 8, b''
    while pos < len(data):
        length, tag = struct.unpack('>I4s', data[pos:pos + 8])
        if tag == b'IDAT':
            idat += data[pos + 8:pos + 8 + length]
        pos += 12 + length
    raw = np.frombuffer(zlib.decompress(idat), dtype=np.uint8).reshape(height, width + 1)
    assert not raw[:, 0].any(), 'only PNG filter type 0 is expected'
    return raw[:, 1:]


# ---------------- occupancy grid ----------------
def test_slice_height_is_the_lidar_height():
    assert CFG['grid']['slice_z'] == _xacro_value('lidar_z')
    assert CFG['grid']['resolution'] == 0.05


@pytest.mark.parametrize('name', WORLDS)
def test_truth_map_shape_origin_and_classes(name):
    builder = _builder(name)
    grid, meta = gw.truth_map(builder, CFG['grid'])
    res, margin = CFG['grid']['resolution'], CFG['grid']['margin_m']
    size_x, size_y = builder.cfg['size_x'], builder.cfg['size_y']
    assert grid.shape == (round((size_y + 2 * margin) / res), round((size_x + 2 * margin) / res))
    assert meta['origin'] == [-size_x / 2 - margin, -size_y / 2 - margin, 0.0]
    assert set(np.unique(grid)) == {gw.GRID_OCCUPIED, gw.GRID_UNKNOWN, gw.GRID_FREE}

    def cell(x, y):
        col = int((x - meta['origin'][0]) / res)
        row = grid.shape[0] - 1 - int((y - meta['origin'][1]) / res)
        return grid[row, col]

    assert cell(0.0, 0.0) == gw.GRID_FREE                       # robot spawn
    assert cell(size_x / 2, 0.3) == gw.GRID_OCCUPIED            # east outer wall
    assert cell(-size_x / 2, -0.3) == gw.GRID_OCCUPIED          # west outer wall
    assert cell(0.3, size_y / 2) == gw.GRID_OCCUPIED            # north outer wall
    assert cell(size_x / 2 + margin / 2, 0.0) == gw.GRID_UNKNOWN  # outside the building


def test_standing_survivor_is_in_the_slice_and_low_debris_is_not():
    b = gw.WorldBuilder('t', {'size_x': 6.0, 'size_y': 6.0}, CFG['common'], 1)
    b.add_survivor(1.0, 1.0, 0.0, 'standing', False)
    b.add_box((0.4, 0.4, 0.10), (-1.0, -1.0, 0.05, 0, 0, 0), (0.5, 0.5, 0.5))   # 10 cm high
    b.add_box((0.4, 0.4, 0.60), (-1.0, 1.0, 0.30, 0, 0, 0.5), (0.5, 0.5, 0.5))  # rotated, tall
    res = 0.05
    slice_grid, origin = gw.occupancy_grid(b, res, 0.5, 0.20, 0.20)
    body_grid, _ = gw.occupancy_grid(b, res, 0.5, 0.0, 0.30)

    def cell(grid, x, y):
        return grid[grid.shape[0] - 1 - int((y - origin[1]) / res), int((x - origin[0]) / res)]

    assert cell(slice_grid, 1.0, 1.09) == gw.GRID_OCCUPIED      # a leg of the survivor
    assert cell(slice_grid, -1.0, -1.0) == gw.GRID_FREE         # debris below the LiDAR plane
    assert cell(body_grid, -1.0, -1.0) == gw.GRID_OCCUPIED      # but the robot would hit it
    assert cell(slice_grid, -1.0, 1.0) == gw.GRID_OCCUPIED
    # a 0.4 m square covers 64 cells; a rotated one must cover about as many
    region = slice_grid[grid_rows(slice_grid, origin, res, 0.6, 1.4), :][:, 40:60]  # x -1.5..-0.5
    assert 50 <= np.count_nonzero(region == gw.GRID_OCCUPIED) <= 80


def grid_rows(grid, origin, res, y_lo, y_hi):
    top = grid.shape[0] - 1 - int((y_hi - origin[1]) / res)
    bottom = grid.shape[0] - 1 - int((y_lo - origin[1]) / res)
    return slice(top, bottom + 1)


@pytest.mark.parametrize('name', WORLDS)
def test_committed_truth_maps_match_the_generator(name):
    grid, meta = gw.truth_map(_builder(name), CFG['grid'])
    committed = _read_png_gray(os.path.join(PKG, 'worlds', name + '_truth_map.png'))
    assert np.array_equal(committed, grid), 'worlds/ is stale: re-run generate_worlds'
    with open(os.path.join(PKG, 'worlds', name + '_truth_map.yaml')) as f:
        assert yaml.safe_load(f) == meta


# ---------------- covariances ----------------
with open(os.path.join(PKG, 'config', 'sensor_covariances.yaml')) as _f:
    COV = yaml.safe_load(_f)


def test_imu_variances_are_the_configured_noise_squared():
    assert COV['imu']['angular_velocity_variance'] == pytest.approx(
        _xacro_value('imu_gyro_noise_stddev') ** 2)
    assert COV['imu']['linear_acceleration_variance'] == pytest.approx(
        _xacro_value('imu_accel_noise_stddev') ** 2)


def test_covariance_matrices_are_well_formed():
    orientation, gyro, accel = imu_covariances(COV['imu'])
    assert orientation[0] == -1.0                 # orientation marked "not provided"
    assert gyro[0] == gyro[4] == gyro[8] == COV['imu']['angular_velocity_variance']
    assert all(len(m) == 9 for m in (orientation, gyro, accel))
    pose, twist = odom_covariances(COV['odom'])
    assert len(pose) == len(twist) == 36
    assert twist[0] == COV['odom']['vx_variance'] and twist[35] == COV['odom']['vyaw_variance']
    assert all(twist[i * 7] > 0 and pose[i * 7] > 0 for i in range(6))
    assert all(v == 0 for i, v in enumerate(twist) if i % 7)
