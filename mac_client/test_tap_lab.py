"""Offline verification; no BLE device or physical action required."""
import unittest
from tap_lab import DEFAULT, Replay, WIRE, NAMES, events

class ReplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.replay = Replay()

    def trace(self, strikes):
        # +/-4 g -> 0.000122 g/LSB: 8197 LSB = 1.0 g rest, 21311 LSB = 2.6 g tap.
        rows = [[0, 0, 8197, 0] for _ in range(900)]
        for start, duration in strikes:
            for i in range(start, start + duration): rows[i][2] = 21311
        return {'samples': rows, 'gaps': 0}

    def test_single_and_double(self):
        for strikes, expected in [([(300,2)], ['single']), ([(300,2),(385,2)], ['double'])]:
            d = self.trace(strikes)
            out = self.replay.run(d, DEFAULT)
            self.assertEqual([e['kind'] for e in events(out)], expected)
            for row, flag in zip(d['samples'], out): row[3] = flag
            self.assertEqual(self.replay.run(d, DEFAULT), out)

    def test_config_changes_same_trace(self):
        d = self.trace([(300,50)])
        self.assertEqual(events(self.replay.run(d, DEFAULT)), [])
        cfg = dict(DEFAULT, max_pulse_ms=140, gravity_alpha=.001)
        self.assertEqual([e['kind'] for e in events(self.replay.run(d, cfg))], ['single'])

    def test_gap_breaks_pair(self):
        d = self.trace([(300,2),(650,2)]); d['samples'][400][3] = 64
        out = self.replay.run(d, DEFAULT)
        self.assertEqual([e['kind'] for e in events(out)], ['single'])
        self.assertTrue(out[400] & 64)

    def test_invalid_config(self):
        d = self.trace([])
        for cfg in [dict(DEFAULT, peak_g=float('nan')), dict(DEFAULT, max_pulse_ms=-1),
                    dict(DEFAULT, min_gap_ms=400)]:
            with self.assertRaises(ValueError): self.replay.run(d, cfg)

    def test_config_wire(self):
        p = WIRE.pack(*(DEFAULT[k] for k in NAMES))
        self.assertEqual(len(p), 40)
        cfg = dict(zip(NAMES, WIRE.unpack(p)))
        d = self.trace([(300,2)])
        self.assertEqual(self.replay.run(d, cfg), self.replay.run(d, DEFAULT))

if __name__ == '__main__': unittest.main()
