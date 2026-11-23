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
        self.in_view = 1                  # frames in which the point was visible to the camera
        self.state = CANDIDATE

    @property
    def confidence(self):
        return self.confidence_sum / self.hits

    @property
    def hit_rate(self):
        return self.hits / max(self.in_view, 1)


class Tracker:

    def __init__(self, radius=1.0, confirm_hits=5, confirm_views=2, view_separation=0.75,
                 expire_s=60.0, min_hit_rate=0.0, rate_after_views=20, confirm_confidence=0.0,
                 merge_radius=0.0):
        self.radius = radius
        self.confirm_hits = confirm_hits
        self.confirm_views = confirm_views
        self.view_separation = view_separation
        self.expire_s = expire_s
        # A person is detected in most frames that see them; a look-alike obstacle only now
        # and then. Once a track has been in view for `rate_after_views` frames it must have
        # been detected in at least `min_hit_rate` of them (0 switches the rule off).
        self.min_hit_rate = min_hit_rate
        self.rate_after_views = rate_after_views
        # Mean detection confidence a track needs to be (and stay) confirmed, and the distance
        # within which only the strongest confirmed track is reported (see suppressed()).
        self.confirm_confidence = confirm_confidence
        self.merge_radius = merge_radius
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
        self._reevaluate(track)
        return track

    def count_view(self, track):
        """The point of `track` was visible to the camera in this frame (detected or not)."""
        track.in_view += 1
        self._reevaluate(track)

    def _quality_ok(self, track):
        rate_ok = (self.min_hit_rate <= 0.0 or track.in_view < self.rate_after_views
                   or track.hit_rate >= self.min_hit_rate)
        return rate_ok and track.confidence >= self.confirm_confidence

    def _reevaluate(self, track):
        """Confirm a candidate that qualifies; demote a confirmed track that stops qualifying."""
        eligible = (track.hits >= self.confirm_hits and len(track.views) >= self.confirm_views
                    and self._quality_ok(track))
        if track.state == CANDIDATE and eligible:
            track.state = CONFIRMED
        elif track.state == CONFIRMED and not self._quality_ok(track):
            track.state = CANDIDATE

    def suppressed(self):
        """Ids of confirmed tracks that have a confirmed track with more hits within merge_radius.

        One survivor can leave several confirmed tracks (a lying person is 1.5 m long and boxes
        cover legs or torso). Only the track with the most hits is reported as a pin.
        """
        if self.merge_radius <= 0.0:
            return set()
        kept, out = [], set()
        for t in sorted(self.confirmed(), key=lambda t: -t.hits):
            if any(math.hypot(t.x - k.x, t.y - k.y) <= self.merge_radius for k in kept):
                out.add(t.id)
            else:
                kept.append(t)
        return out

    def expire(self, now):
        """Drop unconfirmed candidates not seen for expire_s. Returns the dropped tracks."""
        dropped = [t for t in self.tracks
                   if t.state == CANDIDATE and now - t.last_seen > self.expire_s]
        if dropped:
            self.tracks = [t for t in self.tracks if t not in dropped]
        return dropped

    def confirmed(self):
        return [t for t in self.tracks if t.state == CONFIRMED]
