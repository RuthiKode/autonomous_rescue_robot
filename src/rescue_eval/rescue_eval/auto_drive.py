"""Drive a planned tour of a world: a scripted stand-in for teleoperation.

  ros2 run rescue_eval auto_drive --world rooms

Run it against a simulation that is already up, with cmd_vel_record running, to
produce a drive file without a person at the keyboard. The tour is planned on
the world generator's geometry and followed with Gazebo's true pose, so this is
test tooling and not part of the robot: it knows the map in advance.
"""
import argparse
import heapq
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rescue_simulation import generate_worlds as gw
from scipy import ndimage
import yaml

from rescue_eval.driving import DriveNode
from rescue_eval.sim_runner import spin_until


class Planner:
    """Grid planner over the footprint of everything the robot body can hit."""

    def __init__(self, world, cfg):
        gen_cfg = gw.load_config()
        builder = gw.build(world, dict(gen_cfg['worlds'][world]), gen_cfg['common'],
                           gen_cfg['seed'])
        self.res = cfg['grid_resolution']
        grid, self.origin = gw.occupancy_grid(builder, self.res, 0.5, 0.0, cfg['robot_height_m'])
        self.height = grid.shape[0]
        blocked = grid != gw.GRID_FREE
        self.clearance = ndimage.distance_transform_edt(~blocked) * self.res
        self.inflation = cfg.get('inflation_by_world', {}).get(world, cfg['inflation_m'])
        self.free = self.clearance > self.inflation
        self.check_free = self.clearance > cfg['check_inflation_m']
        self.weight = cfg['clearance_weight']
        self.spawn = (builder.spawn['x'], builder.spawn['y'])

    def cell(self, x, y):
        col = int((x - self.origin[0]) / self.res)
        row = self.height - 1 - int((y - self.origin[1]) / self.res)
        return row, col

    def point(self, row, col):
        return (self.origin[0] + (col + 0.5) * self.res,
                self.origin[1] + (self.height - 1 - row + 0.5) * self.res)

    def _component(self, free):
        labels, _ = ndimage.label(free, structure=np.ones((3, 3)))
        start = labels[self.cell(*self.spawn)]
        if start == 0:
            raise RuntimeError('the spawn is not in free space at this inflation')
        return labels == start

    def reachable(self):
        return self._component(self.free)

    def reachable_fraction(self):
        """Cells reachable at the planning inflation / cells reachable at the check inflation."""
        return float(np.count_nonzero(self.reachable())
                     / np.count_nonzero(self._component(self.check_free)))

    def goals(self, count, min_spacing):
        """Farthest-point sampling over reachable cells, starting from the spawn."""
        reach = self.reachable()
        rows, cols = np.nonzero(reach)
        pts = np.array([self.point(r, c) for r, c in zip(rows, cols)])
        dist = np.hypot(pts[:, 0] - self.spawn[0], pts[:, 1] - self.spawn[1])
        chosen = []
        for _ in range(count):
            i = int(np.argmax(dist))
            if dist[i] < min_spacing:
                break
            chosen.append((float(pts[i, 0]), float(pts[i, 1])))
            dist = np.minimum(dist, np.hypot(pts[:, 0] - pts[i, 0], pts[:, 1] - pts[i, 1]))
        return chosen

    def path(self, a, b):
        """A* from world point a to b. Returns world points, or None."""
        start, goal = self.cell(*a), self.cell(*b)
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        best, parent = {start: 0.0}, {}
        queue = [(0.0, start)]
        while queue:
            _, cur = heapq.heappop(queue)
            if cur == goal:
                break
            for dr, dc, step in moves:
                nxt = (cur[0] + dr, cur[1] + dc)
                if not self.free[nxt]:
                    continue
                cost = best[cur] + step * (1.0 + self.weight / self.clearance[nxt])
                if cost < best.get(nxt, float('inf')):
                    best[nxt], parent[nxt] = cost, cur
                    h = math.hypot(nxt[0] - goal[0], nxt[1] - goal[1])
                    heapq.heappush(queue, (cost + h, nxt))
        if goal not in parent and goal != start:
            return None
        cells = [goal]
        while cells[-1] != start:
            cells.append(parent[cells[-1]])
        return [self.point(r, c) for r, c in reversed(cells)]

    def _clear_line(self, a, b):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res / 2)))
        return all(self.free[self.cell(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)]
                   for i in range(n + 1))

    def simplify(self, pts, max_segment=2.0):
        """Drop intermediate points while the straight line between the kept ones is clear."""
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not (self._clear_line(pts[i], pts[j])
                                     and math.hypot(pts[j][0] - pts[i][0],
                                                    pts[j][1] - pts[i][1]) <= max_segment):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def tour(self, count, min_spacing):
        """Waypoints visiting the goals nearest-first and returning to the spawn."""
        remaining = self.goals(count, min_spacing)
        here, waypoints, visited = self.spawn, [], []
        while remaining:
            nxt = min(remaining, key=lambda g: math.hypot(g[0] - here[0], g[1] - here[1]))
            remaining.remove(nxt)
            leg = self.path(here, nxt)
            if leg is None:
                continue
            waypoints += self.simplify(leg)[1:]
            visited.append(nxt)
            here = nxt
        back = self.path(here, self.spawn)
        if back is not None:
            waypoints += self.simplify(back)[1:]
        return waypoints, visited


def main(argv=None):
    share = get_package_share_directory('rescue_eval')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--world', required=True)
    parser.add_argument('--config', default=os.path.join(share, 'config', 'auto_drive.yaml'))
    parser.add_argument('--plan-only', action='store_true', help='print the tour and exit')
    parser.add_argument('--min-subscribers', type=int, default=1,
                        help='/cmd_vel subscribers to wait for before driving'
                             ' (2 when recording: the bridge and the recorder)')
    args, ros_args = parser.parse_known_args(argv)
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    planner = Planner(args.world, cfg)
    waypoints, goals = planner.tour(cfg['goals'][args.world], cfg['min_goal_spacing_m'])
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip([planner.spawn] + waypoints, waypoints))
    print(f'{args.world}: {len(goals)} goals, {len(waypoints)} waypoints, {length:.1f} m')
    print('goals: ' + ', '.join(f'({x:.2f}, {y:.2f})' for x, y in goals))
    fraction = planner.reachable_fraction()
    print(f'inflation {planner.inflation:.2f} m: {100 * fraction:.0f}% of the floor that is'
          f" reachable at {cfg['check_inflation_m']:.2f} m is reachable by this tour")
    required = cfg['min_reachable_fraction'].get(args.world)
    if required is not None and fraction < required:
        print(f'FAIL: below the required {100 * required:.0f}%; a barrier or gap is closed'
              ' to the planner')
        return 1
    if args.plan_only:
        return 0

    rclpy.init(args=ros_args)
    node = DriveNode('auto_drive')
    try:
        if spin_until(node, lambda: node.truth is not None, 60.0) != 'ok':
            print('no /ground_truth/odom: is the simulation running?')
            return 1
        # A command published before a subscriber has connected is lost to that
        # subscriber only. If the simulator bridge connects before the recorder,
        # the robot moves on commands the recording never gets. So wait for all
        # of them, then stand still briefly before the first real command.
        if spin_until(node, lambda: node.cmd_pub.get_subscription_count()
                      >= args.min_subscribers, 30.0) != 'ok':
            print(f'/cmd_vel has {node.cmd_pub.get_subscription_count()} subscriber(s),'
                  f' expected {args.min_subscribers}')
            return 1
        node.stop_and_settle(1.0)
        result = node.follow(waypoints, cfg['follower'])
        node.stop_and_settle(1.0)
        x, y = node.truth.pose.pose.position.x, node.truth.pose.pose.position.y
        print(f'tour {result}; finished at ({x:.2f}, {y:.2f}), sim time {node.sim_time:.1f} s')
        return 0 if result == 'ok' else 1
    except KeyboardInterrupt:
        node.command(0.0, 0.0)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
