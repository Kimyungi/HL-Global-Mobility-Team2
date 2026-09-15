#!/usr/bin/env python3
"""Build a single offline HTML from real C++ planner output; no JS reimplementation."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'src/stack_avoid_v2'
ASSETS = PACKAGE / 'tools/lab'


def build(output, data_path=None):
    core_files = [PACKAGE/'include/stack_avoid_v2/core.hpp', PACKAGE/'src/core.cpp']
    core_hash = hashlib.sha256(b''.join(p.read_bytes() for p in core_files)).hexdigest()
    if data_path is None:
        with tempfile.TemporaryDirectory(prefix='avoid_v2_html_') as tmp:
            binary = Path(tmp)/'export'
            subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic',
                            '-I', str(PACKAGE/'include'), str(PACKAGE/'src/core.cpp'),
                            str(PACKAGE/'tools/export_lab.cpp'), '-o', str(binary)], check=True)
            dataset = Path(tmp)/'data.json'
            subprocess.run([str(binary), str(dataset)], check=True)
            data = json.loads(dataset.read_text())
    else:
        data = json.loads(Path(data_path).read_text())
    data['coreHash'] = core_hash
    fixture_files = [PACKAGE/'tools/export_lab.cpp', PACKAGE/'test/same_vehicle_cases.hpp',
                     PACKAGE/'test/s_curve_layouts.hpp', PACKAGE/'test/vehicle_case_oracle.hpp',
                     PACKAGE/'test/fixtures.hpp']
    data['fixtureHash'] = hashlib.sha256(b''.join(p.read_bytes() for p in fixture_files)).hexdigest()
    data['generated'] = datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')
    result = (ASSETS/'template.html').read_text()
    for token, content in [('__STYLE__', (ASSETS/'style.css').read_text()),
                           ('__SCRIPT__', (ASSETS/'app.js').read_text()),
                           ('__DATA__', json.dumps(data, ensure_ascii=False, separators=(',', ':')).replace('<', '\\u003c'))]:
        assert result.count(token) == 1, token
        result = result.replace(token, content)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(result)
    print(f'HTML: {output} ({output.stat().st_size / 1024**2:.2f} MiB)')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'docs/avoid_v2_path_lab.html')
    parser.add_argument('--data', type=Path, help='Reuse an export from the current C++ source')
    args = parser.parse_args()
    build(args.output, args.data)
