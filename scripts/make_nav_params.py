#!/usr/bin/env python3
"""Derive rescue_navigation/config/nav2_params.yaml from Nav2's default file.

  python3 scripts/make_nav_params.py

Every change is a literal replacement that must match the expected number of times,
so a Nav2 update that moves a line makes this script fail loudly instead of silently
producing a different file. The header of the output lists the changes.
"""
import os

SRC = '/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'
DST = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src',
                   'rescue_navigation', 'config', 'nav2_params.yaml')
text = open(SRC).read()

# Nav2 runs on slam_toolbox's live /map: no map server, no AMCL. Drop the sections of
# that setup first (they also hold two of the three base_footprint settings).
a = text.index('amcl:\n')
b = text.index('bt_navigator:\n')
text = text[:a] + text[b:]
c = text.index('loopback_simulator:\n')
text = text[:c].rstrip('\n') + '\n'
for gone in ('amcl:', 'loopback_simulator:'):
    assert gone not in text

# (old, new, expected count, why), applied in order.
CHANGES = [
    ('    odom_topic: /odom\n', '    odom_topic: /odometry/filtered\n', 1,
     'bt_navigator reads the EKF output, per the topic contract'),
    ('    controller_frequency: 20.0\n',
     '    controller_frequency: 20.0\n    odom_topic: /odometry/filtered\n', 1,
     'controller_server reads its speed from the EKF output; its default, /odom, does not exist '
     'here, which made MPPI believe the robot was always at rest'),
    ('        consider_footprint: false\n', '        consider_footprint: true\n', 1,
     'MPPI cost critic checks the real footprint, as the costmaps do'),
    ('      vx_max: 0.5\n      vx_min: -0.35\n', '      vx_max: 0.4\n      vx_min: -0.2\n', 1,
     'controller limit: slower forward speed and gentler reverse for narrow gaps'),
    ('      ax_max: 3.0\n      ax_min: -3.0\n', '      ax_max: 1.5\n      ax_min: -1.5\n', 1,
     "controller limit: the robot's own acceleration limit (max_linear_acceleration, Xacro)"),
    ('      az_max: 3.5\n', '      az_max: 4.0\n', 1,
     "controller limit: the robot's angular acceleration limit (max_angular_acceleration, Xacro)"),
    ('      wz_max: 1.9\n', '      wz_max: 1.5\n', 1,
     'controller limit: lower turn rate, less slip at wheel_mu 0.12'),
    ('      robot_radius: 0.22\n',
     '      footprint: "[[0.14, 0.14], [0.14, -0.14], [-0.26, -0.14], [-0.26, 0.14]]"\n', 2,
     'both costmaps: the real rectangular footprint (Xacro chassis 0.40 x 0.28, offset -0.06 '
     'from base_link) instead of a 0.22 m circle'),
    ('        inflation_radius: 0.70\n', '        inflation_radius: 0.45\n', 1,
     'inflation radius, local costmap: circumscribed radius 0.30 m plus a 0.15 m margin'
     ' (the default 0.70 swallows the 0.7 m gaps)'),
    ('        inflation_radius: 0.7\n', '        inflation_radius: 0.45\n', 1,
     'inflation radius, global costmap: the same value'),
    ('    max_velocity: [0.5, 0.0, 2.0]\n    min_velocity: [-0.5, 0.0, -2.0]\n',
     '    max_velocity: [0.4, 0.0, 1.5]\n    min_velocity: [-0.2, 0.0, -1.5]\n', 1,
     'velocity smoother limits follow the controller limits'),
    ('    max_accel: [2.5, 0.0, 3.2]\n    max_decel: [-2.5, 0.0, -3.2]\n',
     '    max_accel: [1.5, 0.0, 4.0]\n    max_decel: [-1.5, 0.0, -4.0]\n', 1,
     "velocity smoother accelerations follow the robot's limits"),
    ('    odom_topic: "odom"\n', '    odom_topic: "odometry/filtered"\n', 1,
     'velocity smoother reads the EKF output'),
    ('    base_frame_id: "base_footprint"\n', '    base_frame_id: "base_link"\n', 1,
     'collision monitor: this robot has no base_footprint frame'),
]
applied = []
for old, new, expected, why in CHANGES:
    n = text.count(old)
    assert n == expected, (old, n)
    text = text.replace(old, new)
    applied.append(why)

header = ("# Nav2 parameters for the rescue robot, derived from Nav2 Jazzy's nav2_params.yaml by\n"
          '# scripts/make_nav_params.py. Nav2 runs on slam_toolbox\'s live /map: there is no map\n'
          '# server and no AMCL (docs/PLAN.md, stage 3), so those sections are removed.\n#\n'
          "# Changes from the default (everything else is Nav2's own):\n"
          + ''.join(f'#   - {w}\n' for w in applied) + '#\n')
os.makedirs(os.path.dirname(DST), exist_ok=True)
with open(DST, 'w') as f:
    f.write(header + text)
print('wrote', os.path.normpath(DST), len(text.splitlines()), 'lines,', len(applied), 'changes')
