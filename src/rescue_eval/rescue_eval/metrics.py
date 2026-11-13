"""CSV schema of the evaluation harness."""
import csv
import os

# Facts about the run itself.
RUN_COLUMNS = [
    'run_id', 'timestamp_utc', 'world', 'method', 'seed', 'status', 'drive',
    'sim_duration_s', 'wall_duration_s', 'real_time_factor',
    'n_true_survivors', 'bag_path', 'bag_messages', 'bag_topics_with_data', 'map_path',
]

# Metrics defined in docs/PLAN.md. Each one is filled in by the stage that makes
# it measurable; until then it is written as "nan", never as a made-up number.
# Measured from Phase 2: map_iou, map_accuracy, ate_rmse_m, ate_aligned_rmse_m.
METRIC_COLUMNS = [
    'nav_success_rate', 'map_iou', 'map_accuracy', 'ate_rmse_m', 'ate_aligned_rmse_m',
    'coverage', 'time_to_90_coverage_s',
    'det_precision', 'det_recall', 'duplicate_rate', 'pin_error_m',
    'mission_completed', 'mission_time_s', 'recoveries',
]

CSV_COLUMNS = RUN_COLUMNS + METRIC_COLUMNS


def placeholder_metrics():
    return {name: 'nan' for name in METRIC_COLUMNS}


def append_row(csv_path, row):
    """Append one run to the CSV, writing the header if the file is new."""
    unknown = set(row) - set(CSV_COLUMNS)
    if unknown:
        raise ValueError(f'unknown CSV columns: {sorted(unknown)}')
    os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
    new_file = not os.path.isfile(csv_path) or os.path.getsize(csv_path) == 0
    if not new_file:
        with open(csv_path, newline='') as f:
            header = next(csv.reader(f))
        if header != CSV_COLUMNS:
            raise ValueError(
                f'{csv_path} was written with a different column set. Move it aside'
                ' (it stays readable) and run again to start a file with the current columns.')
    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, restval='nan')
        if new_file:
            writer.writeheader()
        writer.writerow(row)
