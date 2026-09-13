#!/usr/bin/env python3
"""Copy the running GPS node's serial read results using a bounded strace.

Run while the vehicle is stopped. Tracing can delay the GPS process. This tool
never opens the receiver, sends configuration, or publishes ROS commands.
Requires strace and sudo for attaching on Ubuntu with ptrace_scope=1.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys


def find_gps(device):
    owners = []
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            args = proc.joinpath('cmdline').read_bytes().split(b'\0')
            if not any(Path(os.fsdecode(arg)).name == 'stack_gps_node'
                       for arg in args[:2]):
                continue
            fds = [fd.name for fd in proc.joinpath('fd').iterdir()
                   if os.path.realpath(fd) == str(device)]
            if fds:
                owners.append((int(proc.name), fds))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    if len(owners) != 1:
        raise RuntimeError(f'Expected one GPS node owning {device}; found {owners}')
    return owners[0]


def decode_reads(trace):
    chunks = []
    # -xx makes every byte explicit. Handle both complete and resumed reads.
    result = re.compile(r'"((?:\\x[0-9a-fA-F]{2})*)",\s*\d+\)\s*=\s*(\d+)\b')
    for line in trace.splitlines():
        if 'read(' not in line and '<... read resumed>' not in line:
            continue
        match = result.search(line)
        if not match:
            if re.search(r'\)\s*=\s*[1-9]\d*\b', line):
                raise RuntimeError('Unrecognized/truncated successful read: ' + line[:160])
            continue
        chunk = bytes.fromhex(match[1].replace('\\x', ''))
        if len(chunk) != int(match[2]):
            raise RuntimeError('Trace byte count mismatch; refusing incomplete reconstruction.')
        chunks.append(chunk)
    return b''.join(chunks)


def summarize(raw):
    types = Counter()
    quality = Counter()
    bad = []
    gga = []
    valid = 0
    # Capture may begin/end inside a sentence; retain bytes but only inspect
    # newline-terminated sentences starting at a real '$'. Never repair a CRC.
    for line in raw.split(b'\n')[:-1]:
        line = line.rstrip(b'\r')
        if not line.startswith(b'$'):
            continue
        text = line.decode('ascii', errors='backslashreplace')
        match = re.fullmatch(rb'\$([^*]+)\*([0-9A-Fa-f]{2})', line)
        checksum_ok = False
        if match:
            expected = 0
            for byte in match[1]:
                expected ^= byte
            checksum_ok = expected == int(match[2], 16)
        tag = text.split(',', 1)[0]
        types[tag] += 1
        if not checksum_ok:
            bad.append(text)
        else:
            valid += 1
        fields = text.split('*', 1)[0].split(',')
        if tag.endswith('GGA') and len(fields) >= 15:
            record = dict(raw=text, checksum_valid=checksum_ok,
                          utc=fields[1], quality=fields[6], satellites=fields[7],
                          hdop=fields[8], correction_age_s=fields[13] or None,
                          station_id=fields[14] or None)
            gga.append(record)
            if checksum_ok:
                quality[fields[6]] += 1
    return dict(bytes=len(raw), sentence_counts=dict(types),
                checksum_valid=valid, checksum_invalid=len(bad),
                invalid_sentences=bad, valid_gga_quality_counts=dict(quality), gga=gga)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=20, choices=range(1, 61), metavar='1..60')
    parser.add_argument('--analyze', type=Path, help='Analyze saved strace output without attaching')
    args = parser.parse_args()
    if args.analyze:
        raw = decode_reads(args.analyze.read_text())
        print(json.dumps(summarize(raw), ensure_ascii=False, indent=2))
        return 0
    if os.geteuid() != 0:
        raise RuntimeError('Run this capture with sudo (ptrace attachment permission).')
    tracer = shutil.which('strace')
    if not tracer:
        raise RuntimeError('strace is not installed.')
    device = Path('/dev/ttyRover').resolve(strict=True)
    pid, fds = find_gps(device)
    root = Path(__file__).resolve().parents[1]
    output = root / 'drive_logs' / 'gps_audit_20260912' / (
        'nmea_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True)
    command = [tracer, '-f', '-p', str(pid), '-e', 'trace=read',
               '-P', str(device), '-xx', '-s', '65536', '-ttt',
               '-o', str(output / 'reads.strace')]
    print(f'Capturing NMEA: {args.seconds} seconds, GPS PID={pid}, FD={fds}, {device}', flush=True)
    print(f'Output: {output}', flush=True)
    with (output / 'trace.stderr').open('w') as error_log:
        process = subprocess.Popen(command, stderr=error_log, start_new_session=True)
        try:
            process.wait(timeout=args.seconds)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            pass
        finally:
            if process.poll() is None:
                # Signal only the tracer PID. Never signal/kill the GPS node.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
    trace = output / 'reads.strace'
    raw = decode_reads(trace.read_text()) if trace.exists() else b''
    report = summarize(raw)
    report.update(gps_pid=pid, serial_device=str(device), serial_fds=fds,
                  requested_duration_s=args.seconds, tracer_returncode=process.returncode)
    (output / 'nmea.raw').write_bytes(raw)
    (output / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    # Newly created artifacts remain usable by the person invoking sudo.
    if os.environ.get('SUDO_UID', '').isdigit() and os.environ.get('SUDO_GID', '').isdigit():
        for path in [*output.iterdir(), output]:
            os.chown(path, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))
    print('Valid GGA quality counts:', report['valid_gga_quality_counts'])
    print('NMEA checksums: valid=', report['checksum_valid'], 'invalid=', report['checksum_invalid'])
    for record in report['gga'][-3:]:
        print(record['raw'])
        print('  checksum=', record['checksum_valid'], 'correction_age_s=',
              record['correction_age_s'], 'station_id=', record['station_id'])
    if not raw:
        print((output / 'trace.stderr').read_text(), file=sys.stderr)
        raise RuntimeError('No serial read bytes captured. See trace.stderr.')
    if not report['gga']:
        raise RuntimeError('Serial bytes captured, but no complete GGA sentence; inspect nmea.raw.')
    print('NMEA_CAPTURE_COMPLETE', flush=True)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as error:
        print(f'NMEA_CAPTURE_FAILED: {error}', file=sys.stderr)
        sys.exit(1)
