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
| method | world | seed | status | coverage | true survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) | RTF |
|---|---|---|---|---|---|---|---|---|---|---|
| finetuned_yolo | collapsed | 1 | ok | 1.000 | 4 of 4 | 40 | 0.10 | 0.05 | 0.18 | 0.84 |
| finetuned_yolo | rooms | 1 | ok | 0.825 | 3 of 3 | 14 | 0.21 | 0.00 | 0.30 | 0.90 |
| finetuned_yolo | open_plan | 1 | ok | 0.998 | 3 of 3 | 30 | 0.10 | 0.00 | 0.07 | 0.88 |
| finetuned_yolo | collapsed | 2 | ok | 0.999 | 4 of 4 | 42 | 0.10 | 0.10 | 0.25 | 0.90 |
| finetuned_yolo | rooms | 2 | ok | 0.984 | 3 of 3 | 11 | 0.27 | 0.09 | 0.28 | 0.89 |
| finetuned_yolo | open_plan | 2 | ok | 0.998 | 3 of 3 | 30 | 0.10 | 0.03 | 0.17 | 0.85 |

Pipeline runs: round 2 detectors, leave-one-layout-out (results/phase5r2/runs.csv)
| method | world | seed | status | coverage | true survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) | RTF |
|---|---|---|---|---|---|---|---|---|---|---|
| finetuned_yolo_r2 | collapsed | 1 | ok | 0.993 | 4 of 4 | 37 | 0.11 | 0.08 | 0.39 | 0.76 |
| finetuned_yolo_r2 | rooms | 1 | ok | 0.547 | 1 of 3 | 5 | 0.20 | 0.00 | 0.08 | 0.91 |
| finetuned_yolo_r2 | open_plan | 1 | ok | 0.998 | 3 of 3 | 24 | 0.12 | 0.00 | 0.12 | 0.75 |
| finetuned_yolo_r2 | collapsed | 2 | ok | 0.999 | 4 of 4 | 36 | 0.11 | 0.06 | 0.37 | 0.84 |
| finetuned_yolo_r2 | rooms | 2 | ok | 0.738 | 3 of 3 | 12 | 0.25 | 0.08 | 0.34 | 0.87 |
| finetuned_yolo_r2 | open_plan | 2 | ok | 1.000 | 3 of 3 | 28 | 0.11 | 0.04 | 0.09 | 0.85 |

Pipeline runs: all-layouts detector (world seed 42 unseen) (results/phase5all/runs.csv)
| method | world | seed | status | coverage | true survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) | RTF |
|---|---|---|---|---|---|---|---|---|---|---|
| finetuned_yolo_all | collapsed | 1 | ok | 0.999 | 4 of 4 | 41 | 0.10 | 0.05 | 0.33 | 0.89 |
| finetuned_yolo_all | rooms | 1 | ok | 0.931 | 3 of 3 | 27 | 0.11 | 0.07 | 0.32 | 0.85 |
| finetuned_yolo_all | open_plan | 1 | ok | 0.997 | 3 of 3 | 29 | 0.10 | 0.00 | 0.13 | 0.72 |
| finetuned_yolo_all | collapsed | 2 | ok | 1.000 | 4 of 4 | 40 | 0.10 | 0.03 | 0.32 | 0.69 |
| finetuned_yolo_all | rooms | 2 | ok | 0.935 | 3 of 3 | 23 | 0.13 | 0.04 | 0.22 | 0.72 |
| finetuned_yolo_all | open_plan | 2 | ok | 0.996 | 3 of 3 | 44 | 0.07 | 0.05 | 0.12 | 0.67 |

Pipeline runs: round 4 detector (exploration frames), rule off / hit-rate (results/phase5x/runs.csv)
| method | world | seed | status | coverage | true survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) | RTF |
|---|---|---|---|---|---|---|---|---|---|---|
| x_rate0.0 | collapsed | 1 | ok | 0.999 | 4 of 4 | 11 | 0.36 | 0.18 | 0.30 | 0.75 |
| x_rate0.0 | rooms | 1 | ok | 0.938 | 3 of 3 | 5 | 0.60 | 0.00 | 0.31 | 0.74 |
| x_rate0.0 | open_plan | 1 | ok | 0.996 | 3 of 3 | 7 | 0.43 | 0.14 | 0.17 | 0.61 |
| x_rate0.0 | collapsed | 2 | ok | 0.994 | 4 of 4 | 10 | 0.40 | 0.20 | 0.34 | 0.67 |
| x_rate0.0 | rooms | 2 | ok | 0.891 | 3 of 3 | 3 | 1.00 | 0.00 | 0.29 | 0.62 |
| x_rate0.0 | open_plan | 2 | ok | 0.998 | 3 of 3 | 11 | 0.27 | 0.18 | 0.22 | 0.59 |
| x_rate0.25 | collapsed | 1 | startup_timeout | nan | 0 of 4 | n/a | nan | nan | nan | nan |
| x_rate0.25 | rooms | 1 | ok | 0.865 | 3 of 3 | 4 | 0.75 | 0.25 | 0.28 | 0.66 |
| x_rate0.25 | open_plan | 1 | ok | 1.000 | 3 of 3 | 8 | 0.38 | 0.38 | 0.11 | 0.63 |
| x_rate0.25 | collapsed | 2 | ok | 0.999 | 4 of 4 | 9 | 0.44 | 0.22 | 0.31 | 0.64 |
| x_rate0.25 | rooms | 2 | ok | 0.997 | 3 of 3 | 4 | 0.75 | 0.25 | 0.24 | 0.64 |
| x_rate0.25 | open_plan | 2 | ok | 0.988 | 3 of 3 | 4 | 0.75 | 0.25 | 0.27 | 0.65 |

Pipeline runs: round 4 detector + final pin rule (results/phase5y/runs.csv)
| method | world | seed | status | coverage | true survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) | RTF |
|---|---|---|---|---|---|---|---|---|---|---|
| rule_committed | collapsed | 1 | startup_timeout | nan | 0 of 4 | n/a | nan | nan | nan | nan |
| rule_committed | rooms | 1 | ok | 0.912 | 3 of 3 | 3 | 1.00 | 0.00 | 0.31 | 0.83 |
| rule_committed | open_plan | 1 | ok | 0.998 | 3 of 3 | 5 | 0.60 | 0.00 | 0.12 | 0.75 |
| rule_committed | collapsed | 2 | ok | 0.984 | 4 of 4 | 7 | 0.57 | 0.00 | 0.34 | 0.83 |
| rule_committed | rooms | 2 | ok | 0.905 | 3 of 3 | 3 | 1.00 | 0.00 | 0.37 | 0.87 |
| rule_committed | open_plan | 2 | ok | 0.999 | 3 of 3 | 5 | 0.60 | 0.00 | 0.19 | 0.79 |
| rule_unseen21 | collapsed | 1 | ok | 0.965 | 3 of 4 | 3 | 1.00 | 0.00 | 0.41 | 0.56 |
| rule_unseen21 | rooms | 1 | ok | 0.551 | 0 of 5 | n/a | nan | nan | nan | 0.31 |
| rule_committed | collapsed | 1 | ok | 0.967 | 4 of 4 | 6 | 0.67 | 0.00 | 0.55 | 0.76 |
| rule_unseen21 | open_plan | 1 | ok | 1.000 | 3 of 3 | 4 | 0.75 | 0.00 | 0.30 | 0.82 |
| rule_unseen22 | collapsed | 1 | ok | 0.980 | 3 of 3 | 4 | 0.75 | 0.00 | 0.24 | 0.91 |
| rule_unseen22 | rooms | 1 | ok | 0.944 | 5 of 5 | 5 | 1.00 | 0.00 | 0.39 | 0.90 |
| rule_unseen22 | open_plan | 1 | ok | 1.000 | 5 of 5 | 7 | 0.71 | 0.00 | 0.29 | 0.78 |
