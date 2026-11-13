"""The results CSV must keep one header and one row per run."""
import csv

import pytest

from rescue_eval.metrics import append_row, CSV_COLUMNS, METRIC_COLUMNS, placeholder_metrics


def test_rows_accumulate_under_one_header(tmp_path):
    path = tmp_path / 'nested' / 'runs.csv'
    for seed in (1, 2):
        row = {'run_id': f'r{seed}', 'world': 'rooms', 'seed': seed, 'status': 'ok'}
        row.update(placeholder_metrics())
        append_row(str(path), row)
    with open(path) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2
    assert list(rows[0]) == CSV_COLUMNS
    assert [r['seed'] for r in rows] == ['1', '2']
    assert all(rows[0][m] == 'nan' for m in METRIC_COLUMNS)


def test_unknown_column_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        append_row(str(tmp_path / 'runs.csv'), {'not_a_column': 1})
