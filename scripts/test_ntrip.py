#!/usr/bin/env python3
"""Test NTRIP connection using credentials from secrets.yaml.

Connects to the configured NTRIP caster and prints a live summary of
RTCM corrections received each second.

Usage (from package directory):
    python3 scripts/test_ntrip.py

Or after colcon build:
    ros2 run mr_pinchy_bringup test_ntrip.py

Requires: pip install pygnssutils pyyaml
"""
import argparse
import os
import sys
import time
from collections import defaultdict
from queue import Queue, Empty
from threading import Event

import yaml
from pygnssutils import GNSSNTRIPClient


DEFAULT_SECRETS = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'secrets.yaml',
)

DEFAULT_LAT = 59.204371
DEFAULT_LON = 18.175105
DEFAULT_ALT = 20.6


def main():
    parser = argparse.ArgumentParser(description='Test NTRIP connection')
    parser.add_argument(
        '--secrets', default=DEFAULT_SECRETS, help='Path to secrets.yaml',
    )
    parser.add_argument('--lat', type=float, default=DEFAULT_LAT)
    parser.add_argument('--lon', type=float, default=DEFAULT_LON)
    parser.add_argument('--alt', type=float, default=DEFAULT_ALT)
    args = parser.parse_args()

    secrets_path = os.path.realpath(args.secrets)
    if not os.path.isfile(secrets_path):
        print(f'ERROR: secrets file not found: {secrets_path}', file=sys.stderr)
        sys.exit(1)

    with open(secrets_path) as f:
        params = yaml.safe_load(f)['ntrip_client']['ros__parameters']

    print(f"Connecting to {params['host']}:{params['port']}/{params['mountpoint']}")
    print(f"  user:  {params['username']}")
    print(f"  https: {params.get('use_https', False)}")
    print(f"  pos:   ({args.lat}, {args.lon}, {args.alt}m)")
    print()

    output_queue = Queue()
    stop = Event()

    ntrip = GNSSNTRIPClient()
    streaming = ntrip.run(
        server=params['host'],
        port=int(params['port']),
        mountpoint=params['mountpoint'],
        ntripuser=params['username'],
        ntrippassword=params['password'],
        https=params.get('use_https', False),
        ggamode=1,
        ggainterval=10,
        reflat=args.lat,
        reflon=args.lon,
        refalt=args.alt,
        refsep=0.0,
        output=output_queue,
        stopevent=stop,
    )

    if not streaming:
        print('ERROR: failed to start NTRIP stream', file=sys.stderr)
        sys.exit(1)

    print('Streaming RTCM corrections (Ctrl+C to stop)...\n')

    total_bytes = 0
    total_msgs = 0
    interval_counts = defaultdict(int)
    interval_bytes = 0
    last_report = time.monotonic()

    try:
        while not stop.is_set():
            try:
                raw, parsed = output_queue.get(timeout=1.0)
            except Empty:
                if time.monotonic() - last_report > 2.0:
                    print('  (waiting for data...)')
                    last_report = time.monotonic()
                continue

            nbytes = len(raw)
            total_bytes += nbytes
            interval_bytes += nbytes

            if hasattr(parsed, 'identity'):
                msg_type = parsed.identity
            else:
                msg_type = '?'
            total_msgs += 1
            interval_counts[msg_type] += 1

            now = time.monotonic()
            if now - last_report >= 1.0:
                if interval_counts:
                    parts = ' '.join(
                        f'{mt}({c})' for mt, c in sorted(interval_counts.items())
                    )
                    print(
                        f'  [{total_msgs:>6} msgs | {total_bytes:>8} B]  '
                        f'last 1s: {sum(interval_counts.values())} msgs, '
                        f'{interval_bytes} B  |  {parts}'
                    )
                interval_counts.clear()
                interval_bytes = 0
                last_report = now

    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print(f'\nTotal: {total_msgs} RTCM messages, {total_bytes} bytes')


if __name__ == '__main__':
    main()
