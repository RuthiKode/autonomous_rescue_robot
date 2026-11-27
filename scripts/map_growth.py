#!/usr/bin/env python3
"""Known area of /map over time in a bag (prints every Nth message): python3 scripts/map_growth.py BAG [N]"""
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from reloc_offline import read_topics  # noqa: E402

every = int(sys.argv[2]) if len(sys.argv) > 2 else 5
for i, (_, t, m) in enumerate(read_topics(sys.argv[1], ['/map'])):
    if i % every == 0:
        a = np.array(m.data)
        print(f'{t:7.1f} s  known {float((a >= 0).sum()) * m.info.resolution ** 2:6.1f} m2  '
              f'occupied {float((a >= 65).sum()) * m.info.resolution ** 2:5.1f} m2  size {m.info.width}x{m.info.height}')
