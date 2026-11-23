"""Decisions of the Explore branch, without ROS or py_trees (so they can be tested).

The blacklist and the failure counters live here. The behaviours in behaviours.py only move data
between this object, the frontier selector and Nav2.
"""
import math

GO, RETRY, WAIT, DONE, FAILED = 'go', 'retry', 'wait', 'done', 'failed'


class ExploreLogic:

    def __init__(self, repeat_radius_m=1.0, confirm_empty=3, max_consecutive_failures=8,
                 max_consecutive_recoveries=6, confirm_unreachable=8):
        self.repeat_radius_m = repeat_radius_m
        self.confirm_empty = confirm_empty
        self.confirm_unreachable = confirm_unreachable
        self.unreachable_count = 0
        self.want_clear = False         # set once halfway to "unreachable": clear the costmaps
        self.max_consecutive_failures = max_consecutive_failures
        self.max_consecutive_recoveries = max_consecutive_recoveries
        self.recoveries = 0             # recovery behaviours run
        self.consecutive_recoveries = 0 # ... since a goal was last reached
        self.blacklist = []             # (x, y) goals not to choose again
        self.reached = []               # goals that were reached
        self.goals_sent = 0
        self.goals_failed = 0
        self.consecutive_failures = 0
        self.empty_count = 0
        self.fail_reason = ''
        self.done_reason = ''

    @staticmethod
    def _near(p, points, radius):
        return any(math.hypot(p[0] - q[0], p[1] - q[1]) <= radius for q in points)

    def on_selection(self, found, reason, goal_xy):
        """Decision after a SelectFrontier reply.

        GO     send the goal
        RETRY  the goal was blacklisted, ask again at once
        WAIT   nothing usable yet (no map, or maybe no frontier: ask again later)
        DONE   no frontier left (confirmed by `confirm_empty` empty answers in a row)
        FAILED too many failed goals in a row
        """
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.fail_reason = (f'{self.consecutive_failures} goals failed in a row'
                                f' ({self.goals_failed} of {self.goals_sent} in all)')
            return FAILED
        if self.consecutive_recoveries >= self.max_consecutive_recoveries:
            self.fail_reason = (f'{self.consecutive_recoveries} recoveries in a row without'
                                f' reaching a goal ({self.recoveries} in all)')
            return FAILED
        if not found:
            if reason in ('no_map', 'no_pose'):
                return WAIT
            if reason == 'no_frontiers':
                self.empty_count += 1
                self.unreachable_count = 0
                if self.empty_count >= self.confirm_empty:
                    self.done_reason = 'no frontier cluster above the minimum size is left'
                    return DONE
                return WAIT
            # Frontiers exist but none has a path. A stale costmap or a passing obstacle can cause
            # that, so wait longer than for an empty map and clear the costmaps once on the way.
            self.unreachable_count += 1
            if self.unreachable_count == self.confirm_unreachable // 2:
                self.want_clear = True
            if self.unreachable_count >= self.confirm_unreachable:
                self.done_reason = 'no remaining frontier is reachable'
                return DONE
            return WAIT
        self.empty_count = self.unreachable_count = 0
        if self._near(goal_xy, self.reached, self.repeat_radius_m):
            # Reaching this spot did not resolve its frontier (the unknown cells beyond are out
            # of sensor reach), so choosing it again would loop forever.
            self.blacklist.append(goal_xy)
            return RETRY
        return GO

    def on_goal_sent(self):
        self.goals_sent += 1

    def on_result(self, goal_xy, succeeded, interrupted=False):
        """Record how a goal ended. `interrupted` = a recovery cancelled it, so the goal itself is
        not at fault: it is neither blacklisted nor counted as failed."""
        if interrupted:
            return
        if succeeded:
            self.reached.append(goal_xy)
            self.consecutive_failures = 0
            self.consecutive_recoveries = 0
        else:
            self.blacklist.append(goal_xy)
            self.goals_failed += 1
            self.consecutive_failures += 1

    def on_recovery(self):
        self.recoveries += 1
        self.consecutive_recoveries += 1


class VerifyLogic:
    """Which pin to look at next. A pin is a track that is confirmed, or a candidate with at least
    `min_hits` hits; each gets at most `max_attempts` visits, so a pin that stays unresolved
    (not enough close-range frames, or no viewpoint) cannot hold the robot forever."""

    def __init__(self, min_hits=12, max_attempts=2):
        self.min_hits = min_hits
        self.max_attempts = max_attempts
        self.attempts = {}
        self.visits = 0

    def pick(self, tracks, robot_xy):
        """tracks: iterable of (id, x, y, state, hits). Returns the nearest eligible one or None."""
        eligible = [t for t in tracks
                    if self.attempts.get(t[0], 0) < self.max_attempts
                    and (t[3] == 'confirmed' or (t[3] == 'candidate' and t[4] >= self.min_hits))]
        if not eligible:
            return None
        return min(eligible, key=lambda t: math.hypot(t[1] - robot_xy[0], t[2] - robot_xy[1]))

    def on_visit(self, track_id):
        self.attempts[track_id] = self.attempts.get(track_id, 0) + 1
        self.visits += 1
