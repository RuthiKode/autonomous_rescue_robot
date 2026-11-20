Held-out frame scores, conf >= 0.30, IoU >= 0.5 (all test frames of the held-out layout)
| model | held-out | pretrained P / R | fine-tuned P / R | tp / fp / fn |
|---|---|---|---|---|
| round 1 | collapsed | nan / 0.00 | 0.58 / 0.92 | 425 / 312 / 39 |
| round 1 | rooms | nan / 0.00 | 0.93 / 0.96 | 194 / 14 / 9 |
| round 1 | open_plan | nan / 0.00 | 0.93 / 0.94 | 559 / 40 / 36 |
| round 2 (+3 worlds) | collapsed | nan / 0.00 | 0.56 / 0.94 | 437 / 340 / 27 |
| round 2 (+3 worlds) | rooms | nan / 0.00 | 0.92 / 0.94 | 190 / 16 / 13 |
| round 2 (+3 worlds) | open_plan | nan / 0.00 | 0.93 / 0.96 | 571 / 45 / 24 |
| all layouts, world seed 42 held out | all | nan / 0.00 | 0.95 / 0.98 | 194 / 11 / 4 |

Confidence sweep, round 1, fine-tuned
| held-out | conf 0.30 | conf 0.50 | conf 0.70 |
|---|---|---|---|
| collapsed | 0.58 / 0.92 | 0.68 / 0.90 | 0.80 / 0.86 |
| rooms | 0.93 / 0.96 | 0.94 / 0.93 | 0.97 / 0.91 |
| open_plan | 0.93 / 0.94 | 0.95 / 0.92 | 0.96 / 0.86 |

Round 1, recall by group (fine-tuned, held-out layouts)
| held-out | posture=standing | posture=sitting | posture=lying | occluded=False | occluded=True | height=<20px | height=20-60px | height=>=60px |
|---|---|---|---|---|---|---|---|---|
| collapsed | 0.88 (n=163) | 0.94 (n=109) | 0.95 (n=151) | 0.91 (n=267) | 0.93 (n=156) | 0.73 (n=33) | 0.92 (n=96) | 0.93 (n=335) |
| rooms | 0.89 (n=55) | 1.00 (n=49) | 0.99 (n=73) | 0.98 (n=112) | 0.92 (n=65) | 1.00 (n=8) | 0.95 (n=22) | 0.95 (n=173) |
| open_plan | 0.95 (n=189) | 0.95 (n=150) | 0.94 (n=203) | 0.95 (n=316) | 0.94 (n=226) | 0.86 (n=83) | 0.93 (n=154) | 0.96 (n=358) |

Pipeline runs: round 1 detectors, leave-one-layout-out (results/phase5/runs.csv)
| world | seed | coverage | true survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) | RTF |
|---|---|---|---|---|---|---|---|---|
| collapsed | 1 | 1.000 | 4 of 4 | 40 | 0.10 | 0.05 | 0.18 | 0.84 |
| rooms | 1 | 0.825 | 3 of 3 | 14 | 0.21 | 0.00 | 0.30 | 0.90 |
| open_plan | 1 | 0.998 | 3 of 3 | 30 | 0.10 | 0.00 | 0.07 | 0.88 |
| collapsed | 2 | 0.999 | 4 of 4 | 42 | 0.10 | 0.10 | 0.25 | 0.90 |
| rooms | 2 | 0.984 | 3 of 3 | 11 | 0.27 | 0.09 | 0.28 | 0.89 |
| open_plan | 2 | 0.998 | 3 of 3 | 30 | 0.10 | 0.03 | 0.17 | 0.85 |

Pipeline runs: round 2 detectors, leave-one-layout-out (results/phase5r2/runs.csv)
| world | seed | coverage | true survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) | RTF |
|---|---|---|---|---|---|---|---|---|
| collapsed | 1 | 0.993 | 4 of 4 | 37 | 0.11 | 0.08 | 0.39 | 0.76 |
| rooms | 1 | 0.547 | 1 of 3 | 5 | 0.20 | 0.00 | 0.08 | 0.91 |
| open_plan | 1 | 0.998 | 3 of 3 | 24 | 0.12 | 0.00 | 0.12 | 0.75 |
| collapsed | 2 | 0.999 | 4 of 4 | 36 | 0.11 | 0.06 | 0.37 | 0.84 |
| rooms | 2 | 0.738 | 3 of 3 | 12 | 0.25 | 0.08 | 0.34 | 0.87 |
| open_plan | 2 | 1.000 | 3 of 3 | 28 | 0.11 | 0.04 | 0.09 | 0.85 |

Pipeline runs: all-layouts detector (world seed 42 unseen) (results/phase5all/runs.csv)
| world | seed | coverage | true survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) | RTF |
|---|---|---|---|---|---|---|---|---|
| collapsed | 1 | 0.999 | 4 of 4 | 41 | 0.10 | 0.05 | 0.33 | 0.89 |
| rooms | 1 | 0.931 | 3 of 3 | 27 | 0.11 | 0.07 | 0.32 | 0.85 |
| open_plan | 1 | 0.997 | 3 of 3 | 29 | 0.10 | 0.00 | 0.13 | 0.72 |
| collapsed | 2 | 1.000 | 4 of 4 | 40 | 0.10 | 0.03 | 0.32 | 0.69 |
| rooms | 2 | 0.935 | 3 of 3 | 23 | 0.13 | 0.04 | 0.22 | 0.72 |
| open_plan | 2 | 0.996 | 3 of 3 | 44 | 0.07 | 0.05 | 0.12 | 0.67 |
