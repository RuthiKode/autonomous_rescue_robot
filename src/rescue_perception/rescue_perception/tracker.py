"""Survivor tracks in the map frame. Pure Python, no ROS.

A detection becomes a point in the map frame (see geometry.py). Each point is
associated with the nearest existing track within `radius`; the track position is a
confidence-weighted average of its points and its hit count goes up. A new track starts
as a candidate. It becomes confirmed only after `confirm_hits` hits taken from at least
`confirm_views` robot positions that are at least `view_separation` apart: viewpoint
diversity stops one bad angle (a reflection, a mis-measured depth) from creating a
false survivor. Unconfirmed candidates that have not been seen for `expire_s` are
dropped.
"""
import math

CANDIDATE = 'candidate'
CONFIRMED = 'confirmed'


class Track:

    def __init__(self, track_id, x, y, z, confidence, stamp, viewpoint):
        self.id = track_id
        self.x, self.y, self.z = x, y, z
        self.weight = confidence
        self.confidence_sum = confidence
        self.hits = 1
        self.first_seen = stamp
        self.last_seen = stamp
        self.views = [viewpoint]          # distinct robot positions, each far enough from the rest
        self.state = CANDIDATE

    @property
    def confidence(self):
        return self.confidence_sum / self.hits


class Tracker:

    def __init__(self, radius=1.0, confirm_hits=5, confirm_views=2, view_separation=0.75,
                 expire_s=60.0):
        self.radius = radius
        self.confirm_hits = confirm_hits
        self.confirm_views = confirm_views
        self.view_separation = view_separation
        self.expire_s = expire_s
        self.tracks = []
        self._next_id = 1

    def nearest(self, x, y):
        best, best_d = None, self.radius
        for t in self.tracks:
            d = math.hypot(t.x - x, t.y - y)
            if d <= best_d:
                best, best_d = t, d
        return best

    def update(self, x, y, z, confidence, stamp, viewpoint):
        """Add one observation (map-frame point) and return the track it belongs to."""
        track = self.nearest(x, y)
        if track is None:
            track = Track(self._next_id, x, y, z, confidence, stamp, viewpoint)
            self._next_id += 1
            self.tracks.append(track)
        else:
            w = track.weight + confidence
            track.x = (track.x * track.weight + x * confidence) / w
            track.y = (track.y * track.weight + y * confidence) / w
            track.z = (track.z * track.weight + z * confidence) / w
            track.weight = w
            track.confidence_sum += confidence
            track.hits += 1
            track.last_seen = max(track.last_seen, stamp)
            if all(math.hypot(viewpoint[0] - v[0], viewpoint[1] - v[1]) >= self.view_separation
                   for v in track.views):
                track.views.append(viewpoint)
        if (track.state == CANDIDATE and track.hits >= self.confirm_hits
                and len(track.views) >= self.confirm_views):
            track.state = CONFIRMED
        return track

    def expire(self, now):
        """Drop unconfirmed candidates not seen for expire_s. Returns the dropped tracks."""
        dropped = [t for t in self.tracks
                   if t.state == CANDIDATE and now - t.last_seen > self.expire_s]
        if dropped:
            self.tracks = [t for t in self.tracks if t not in dropped]
        return dropped

    def confirmed(self):
        return [t for t in self.tracks if t.state == CONFIRMED]
