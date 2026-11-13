"""Absolute trajectory error of the SLAM pose (map -> base_link) against Gazebo truth.

The error is computed with evo (https://github.com/MichaelGrupp/evo). Both
trajectories are sampled at the same sim-time stamps, so no time association is
needed. Trajectories are saved in TUM format so `evo_ape tum truth.tum est.tum`
reproduces the numbers.
"""
import math

import numpy as np


class TrajectoryLog:
    """Pairs of (stamp, estimated pose, true pose), each pose as (x, y, yaw) in the world."""

    def __init__(self):
        self.stamps, self.est, self.truth = [], [], []

    def add(self, stamp, est_pose, true_pose):
        self.stamps.append(stamp)
        self.est.append(est_pose)
        self.truth.append(true_pose)

    def __len__(self):
        return len(self.stamps)

    def save_tum(self, est_path, truth_path):
        for path, poses in ((est_path, self.est), (truth_path, self.truth)):
            with open(path, 'w') as f:
                for stamp, (x, y, yaw) in zip(self.stamps, poses):
                    f.write(f'{stamp:.6f} {x:.6f} {y:.6f} 0.000000 0.000000 0.000000'
                            f' {math.sin(yaw / 2):.8f} {math.cos(yaw / 2):.8f}\n')


def map_to_world(pose, spawn):
    """Express a map-frame (x, y, yaw) in the world frame, given the robot spawn pose."""
    c, s = math.cos(spawn[2]), math.sin(spawn[2])
    return (spawn[0] + c * pose[0] - s * pose[1], spawn[1] + s * pose[0] + c * pose[1],
            pose[2] + spawn[2])


def _evo_trajectory(stamps, poses):
    from evo.core.trajectory import PoseTrajectory3D
    xyz = np.array([[x, y, 0.0] for x, y, _ in poses])
    quat_wxyz = np.array([[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)] for _, _, yaw in poses])
    return PoseTrajectory3D(positions_xyz=xyz, orientations_quat_wxyz=quat_wxyz,
                            timestamps=np.array(stamps))


def ate(log):
    """Return dict(rmse, rmse_aligned, n) in metres.

    rmse          translation APE with no alignment: the map frame is tied to the
                  world by the known start pose, so map drift counts as error.
    rmse_aligned  the same after evo's rigid (SE3 Umeyama) alignment, the usual
                  "ATE" in SLAM papers; it shows the shape error on its own.
    """
    if len(log) < 3:
        return {'rmse': float('nan'), 'rmse_aligned': float('nan'), 'n': len(log)}
    import copy

    from evo.core import metrics

    ref = _evo_trajectory(log.stamps, log.truth)
    est = _evo_trajectory(log.stamps, log.est)

    def rmse(estimate):
        ape = metrics.APE(metrics.PoseRelation.translation_part)
        ape.process_data((ref, estimate))
        return float(ape.get_statistic(metrics.StatisticsType.rmse))

    aligned = copy.deepcopy(est)
    aligned.align(ref, correct_scale=False)
    return {'rmse': rmse(est), 'rmse_aligned': rmse(aligned), 'n': len(log)}
