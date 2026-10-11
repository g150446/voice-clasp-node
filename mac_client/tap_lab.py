#!/usr/bin/env python3
"""BLE capture/tuning and offline replay using the firmware C detector.

BLE commands must run in macOS Terminal. Obtain explicit readiness before
capture; one invocation performs one trial. check/set/replay/sweep need no tap.
"""
import argparse
import asyncio
import ctypes as C
import itertools
import json
from pathlib import Path
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
RX = '00000003-0000-1000-8000-00805f9b34fb'
TX = '00000002-0000-1000-8000-00805f9b34fb'
FLOATS = ['quiet_g', 'peak_g', 'edge_g', 'dominance', 'gravity_alpha', 'noise_multiplier']
TIMES = ['pre_quiet_ms', 'max_rise_ms', 'post_quiet_ms', 'max_pulse_ms',
         'min_gap_ms', 'max_gap_ms', 'cooldown_ms', 'suppress_ms']
NAMES = FLOATS + TIMES
DEFAULT = dict(zip(NAMES, [.5, 1., .3, 0., .015, 4., 15, 15, 20, 40, 80, 350, 700, 1500]))
WIRE = struct.Struct('<6f8H')

class Config(C.Structure):
    _fields_ = [(k, C.c_float) for k in FLOATS] + [(k, C.c_uint16) for k in TIMES]


def validate(config):
    if set(config) != set(NAMES):
        raise ValueError('Config must contain exactly: ' + ', '.join(NAMES))
    # Validate the original numbers before ctypes can wrap an integer.
    for k in TIMES:
        if not isinstance(config[k], int) or not 0 <= config[k] <= 65535:
            raise ValueError('Invalid integer: ' + k)
    return Config(**config)


class Replay:
    def __init__(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='harness-tap-replay-')
        lib = Path(self.tmp.name) / 'detector.so'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                        '-ffp-contract=off', '-I' + str(ROOT / 'nordic-main/src'),
                        str(ROOT / 'nordic-main/tests/tap_replay_bridge.c'),
                        str(ROOT / 'nordic-main/src/tap_processing.c'),
                        str(ROOT / 'nordic-main/src/tap_detector.c'), '-lm', '-o', str(lib)], check=True)
        self.lib = C.CDLL(str(lib))
        self.lib.tap_replay.argtypes = [C.POINTER(C.c_int16), C.POINTER(C.c_uint16), C.c_size_t,
                                       C.POINTER(Config), C.POINTER(C.c_uint16)]
        self.lib.tap_replay.restype = C.c_int

    def run(self, data, config):
        rows = data['samples']
        raw = (C.c_int16 * (3*len(rows)))(*(v for row in rows for v in row[:3]))
        flags = (C.c_uint16 * len(rows))(*(row[3] for row in rows))
        out = (C.c_uint16 * len(rows))()
        cfg = validate(config)
        if self.lib.tap_replay(raw, flags, len(rows), C.byref(cfg), out) != 0:
            raise ValueError('Firmware rejects this config')
        return list(out)


def events(flags):
    return [{'sample': i, 'seconds': round(i/416, 4), 'kind': 'single' if f & 16 else 'double'}
            for i, f in enumerate(flags) if f & 48]


def inspect(data, flags):
    starts = [i for i, f in enumerate(flags) if f & 1 and (i == 0 or not flags[i-1] & 1)]
    rejected = [i for i, f in enumerate(flags) if f & 2 and (i == 0 or not flags[i-1] & 2)]
    print(json.dumps({'events': events(flags), 'candidate_samples': starts,
                      'width_rejection_samples': rejected,
                      'saturated_samples': sum(bool(r[3] & 128) for r in data['samples']),
                      'fifo_gaps': data['gaps']}, indent=2))


async def ble(args):
    from device_debug import DebugControl
    from bleak import BleakClient, BleakScanner
    queue = asyncio.Queue()
    tap_events = []
    debug = DebugControl()

    def notify(_, packet):
        p = bytes(packet)
        if p[:2] != b'\x00\x55' or len(p) < 3:
            return
        if p[2] in (0xD6, 0xD7): queue.put_nowait(p)
        if p[2] in (0x14, 0x12):
            tap_events.append({'kind': 'single' if p[2] == 0x14 else 'double',
                               'host_time': asyncio.get_running_loop().time()})
            print('Notification:', tap_events[-1]['kind'], flush=True)

    print('Scanning for HarnessNode...', flush=True)
    device = await BleakScanner.find_device_by_name('HarnessNode', timeout=15)
    if not device: raise RuntimeError('HarnessNode not found')
    print(f'Found {device.name} [{device.address}]', flush=True)
    async with BleakClient(device) as client:
        await client.start_notify(TX, debug.wrap(notify))
        print('Connected; notifications enabled.', flush=True)
        if args.command == 'mode':
            op = 1 if args.enable else 2 if args.disable else 0
            if op == 1:
                async with debug.session(client):
                    print(await debug.mode(client, 0), flush=True)
                    print('Keeping this connection open for diagnostics; Ctrl-C restores normal mode.', flush=True)
                    await asyncio.sleep(args.seconds)
            else:
                print(await debug.mode(client, op), flush=True)
            return
        async with debug.session(client):
            if client.mtu_size < 141: raise RuntimeError('Capture transfer requires ATT MTU >= 141')

            async def request(command, code, op=None, retry=False):
                # Read-only pulls may retry; arm and set are never duplicated.
                for attempt in range(3 if retry else 1):
                    await client.write_gatt_char(RX, command, response=True)
                    try:
                        while True:
                            p = await asyncio.wait_for(queue.get(), timeout=3)
                            if p[2] == code and (op is None or p[3] == op): return p
                    except asyncio.TimeoutError:
                        if attempt == (2 if retry else 0): raise

            p = await request(b'\x54\x00', 0xD7, retry=True)
            if len(p) != 44 or p[3]: raise RuntimeError('Config read failed')
            config = dict(zip(NAMES, WIRE.unpack(p[4:])))
            print('Current config:', json.dumps(config), flush=True)
            expected_config = None
            if getattr(args, 'expect_config', None):
                expected_config = json.loads(Path(args.expect_config).read_text())
                validate(expected_config)
                if WIRE.pack(*(expected_config[k] for k in NAMES)) != p[4:]:
                    raise RuntimeError('Expected config mismatch; no countdown or physical trial started')
            if args.command == 'set':
                if args.defaults: cmd = b'\x54\x02'
                else:
                    config = json.loads(Path(args.config).read_text())
                    validate(config)
                    cmd = b'\x54\x01' + WIRE.pack(*(config[k] for k in NAMES))
                p = await request(cmd, 0xD7)
                if p[3]: raise RuntimeError(f'Config rejected: status={p[3]}')
                print('Applied (RAM only):', dict(zip(NAMES, WIRE.unpack(p[4:]))), flush=True)
                return

            async def status(op=0, count=0):
                cmd = bytes([0x53, op]) + (struct.pack('<H', count) if op else b'')
                p = await request(cmd, 0xD6, op, retry=op == 0)
                if len(p) != 58 or p[4]: raise RuntimeError(f'Capture status failed: {p.hex()}')
                ident, target, captured, hz, gaps = struct.unpack_from('<4HI', p, 5)
                return dict(id=ident, target=target, captured=captured, hz=hz, gaps=gaps, active=bool(p[17]), config=dict(zip(NAMES, WIRE.unpack(p[18:]))))

            if args.command == 'check':
                print('Passive buffer/transfer check: no physical action required.', flush=True)
                armed = await status(1, 416)
                config_before = armed['config']
                cue_time = None
                await asyncio.sleep(1.3)
            else:
                print(f'ONE trial ({args.kind}). After GO: ' +
                      {'single': 'tap once, then remain still.',
                       'double': 'tap twice about 0.2 seconds apart, then remain still.',
                       'motion': 'move your wrist normally without tapping.'}[args.kind], flush=True)
                for n in range(5, 0, -1):
                    print(n, flush=True); await asyncio.sleep(1)
                armed = await status(1, args.seconds*416)
                config_before = armed['config']
                if expected_config and WIRE.pack(*(expected_config[k] for k in NAMES)) != WIRE.pack(*(config_before[k] for k in NAMES)):
                    raise RuntimeError('Config changed before capture; no GO cue issued')
                print('Recording baseline for 1 second; wait for GO.', flush=True)
                await asyncio.sleep(1)
                cue_time = asyncio.get_running_loop().time()
                print('GO', flush=True)
                sound = await asyncio.create_subprocess_exec('afplay', '/System/Library/Sounds/Glass.aiff')
                await asyncio.sleep(args.seconds - 1)
                await sound.wait()
                print('Physical trial ended. Remain still; downloading saved waveform...', flush=True)
            for _ in range(30):
                state = await status()
                if state['id'] != armed['id']: raise RuntimeError('Capture overwritten')
                if not state['active']: break
                await asyncio.sleep(.2)
            else: raise RuntimeError('Capture did not finish')
            if state['captured'] != armed['target']:
                raise RuntimeError(f'Incomplete capture: {state}')
            rows = []
            while len(rows) < state['captured']:
                offset = len(rows)
                p = await request(b'\x53\x02' + struct.pack('<H', offset), 0xD6, 2, retry=True)
                if len(p) < 10 or p[4]: raise RuntimeError('Chunk rejected')
                ident, received, count = struct.unpack_from('<HHB', p, 5)
                if received != offset:
                    # A delayed duplicate of a retried pull; rerequest current offset.
                    continue
                if ident != armed['id'] or not count or len(p) != 10+count*8:
                    raise RuntimeError('Invalid capture chunk')
                rows.extend(struct.unpack_from('<3hH', p, 10+8*i) for i in range(count))
            data = {'schema': 1, 'firmware': '0.0.111+', 'sample_hz': state['hz'],
                    'raw_to_g': .000122, 'kind': getattr(args, 'kind', 'passive'), 'config': config_before,
                    'posture': getattr(args, 'posture', 'unspecified'),
                    'capture_id': state['id'], 'gaps': state['gaps'],
                    'cue_delay_seconds': 1 if cue_time is not None else None, 'host_cue_time': cue_time,
                    'notifications': tap_events, 'samples': rows}
            Path(args.output).write_text(json.dumps(data))
            print(f'Saved {len(rows)} samples: {args.output}', flush=True)
            replay = Replay(); result = replay.run(data, config_before)
            mismatch = [i for i, (row, flag) in enumerate(zip(rows, result)) if row[3] != flag]
            inspect(data, result)
            if state['gaps']: raise RuntimeError('FIFO gaps: retain data for diagnosis, do not use for tuning')
            if mismatch: raise RuntimeError(f'Firmware/PC replay differs at {mismatch[:10]}')
            print('PASS: complete waveform; PC replay matches every firmware state flag.', flush=True)
            if args.command == 'check':
                print('PASS: passive capture/transfer check; no physical trial started.', flush=True)
                return
            expected = [] if args.kind == 'motion' else [args.kind]
            observed = [e['kind'] for e in events(result)]
            print(f'Tap result: expected={expected} observed={observed}', flush=True)
            print('Trial finished. Wait for the agent before another trial.', flush=True)


def offline(args):
    datasets = [json.loads(Path(p).read_text()) for p in args.files]
    for d in datasets:
        if d['sample_hz'] != 416 or d['gaps']: raise ValueError('Cannot tune a trace with gaps/wrong ODR')
    replay = Replay()
    if args.command == 'replay':
        for path, data in zip(args.files, datasets):
            cfg = json.loads(Path(args.config).read_text()) if args.config else data['config']
            flags = replay.run(data, cfg)
            print(path); inspect(data, flags)
            if not args.config:
                if flags != [r[3] for r in data['samples']]: raise RuntimeError('Replay mismatch')
                print('PASS: exact firmware replay')
        return
    if not any(d['kind'] == 'motion' for d in datasets):
        raise ValueError('Sweep requires a no-tap motion trace to check false detections')
    if not all(any(d['kind'] == kind for d in datasets) for kind in ('single', 'double')):
        raise ValueError('Sweep requires both single and double traces')
    winners = []
    base = datasets[0]['config']
    for width, quiet, post, alpha in itertools.product([40,60,80,120,160,200], [.25,.35,.5], [10,20,30], [.001,.005,.015,.03]):
        cfg = dict(base, max_pulse_ms=width, quiet_g=quiet, post_quiet_ms=post, gravity_alpha=alpha)
        if all([e['kind'] for e in events(replay.run(d, cfg))] ==
               ([] if d['kind'] == 'motion' else [d['kind']]) for d in datasets):
            winners.append(cfg)
    Path(args.output).write_text(json.dumps(winners, indent=2))
    print(f'{len(winners)} candidates saved to {args.output}; verify on new held-out traces before adoption.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    subs = p.add_subparsers(dest='command', required=True)
    q = subs.add_parser('mode'); group = q.add_mutually_exclusive_group()
    group.add_argument('--enable', action='store_true'); group.add_argument('--disable', action='store_true')
    q.add_argument('--seconds', type=int, choices=range(1,601), default=600)
    q = subs.add_parser('check'); q.add_argument('--output', default='/tmp/harness_tap_lab_passive_0111.json')
    q = subs.add_parser('set'); group = q.add_mutually_exclusive_group(required=True)
    group.add_argument('--config'); group.add_argument('--defaults', action='store_true')
    q = subs.add_parser('capture'); q.add_argument('--kind', choices=['single','double','motion'], required=True)
    q.add_argument('--posture', default='unspecified', help='Posture label saved with the waveform')
    q.add_argument('--expect-config', help='Require this RAM config before countdown/GO')
    q.add_argument('--seconds', type=int, choices=range(3,9), default=6); q.add_argument('--output', required=True)
    q = subs.add_parser('replay'); q.add_argument('files', nargs='+'); q.add_argument('--config')
    q = subs.add_parser('sweep'); q.add_argument('files', nargs='+'); q.add_argument('--output', required=True)
    args = p.parse_args()
    if args.command in ('replay', 'sweep'): offline(args)
    else: asyncio.run(ble(args))

if __name__ == '__main__': main()
