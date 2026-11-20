"""Score confirmed survivor pins against the true survivors (docs/PLAN.md, evaluation).

Matching is one-to-one: a confirmed pin is correct if it lies within `radius` of a true
survivor that no other pin has been matched to. Pins are matched in order of increasing
distance, so the closest pairs win.

  precision       correct confirmed pins / all confirmed pins
  recall          true survivors matched by a pin / all true survivors
  duplicate rate  extra confirmed pins that lie within radius of an already matched
                  survivor / all confirmed pins
  pin error       mean distance between matched pins and their true survivors
"""
import math


def score_pins(pins, truth, radius=1.0):
    """pins: [(x, y)] of confirmed pins; truth: [(x, y)]. Returns a dict of metrics."""
    pairs = sorted((math.hypot(px - tx, py - ty), i, j)
                   for i, (px, py) in enumerate(pins) for j, (tx, ty) in enumerate(truth)
                   if math.hypot(px - tx, py - ty) <= radius)
    used_pins, used_truth, matched = set(), set(), []
    for d, i, j in pairs:
        if i not in used_pins and j not in used_truth:
            used_pins.add(i)
            used_truth.add(j)
            matched.append(d)
    duplicates = sum(1 for i in range(len(pins)) if i not in used_pins and any(
        math.hypot(pins[i][0] - truth[j][0], pins[i][1] - truth[j][1]) <= radius
        for j in used_truth))
    n_pins, n_truth = len(pins), len(truth)
    return {
        'pins': n_pins, 'truth': n_truth, 'matched': len(matched),
        'precision': len(matched) / n_pins if n_pins else float('nan'),
        'recall': len(matched) / n_truth if n_truth else float('nan'),
        'duplicate_rate': duplicates / n_pins if n_pins else float('nan'),
        'pin_error_m': sum(matched) / len(matched) if matched else float('nan'),
    }
