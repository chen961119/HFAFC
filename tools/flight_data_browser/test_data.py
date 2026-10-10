"""Numerical, corruption and clock-regression tests; no flight hardware."""
from pathlib import Path
import tempfile
import unittest

import numpy as np

from data import LogError, compute_spectrum, display_series, load_log
from main import export_csv


class DataTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def log(self, text, **options):
        file = self.folder / "flight.txt"
        file.write_text(text, encoding="utf-8-sig")
        return load_log(file, **options)

    def test_microseconds_dedup_and_immutable_arrays(self):
        log = self.log("TimeStamp(us),ROLL_IMU(deg)\n1000000,1\n1020000,2\n1020000,999\n1040000,3\n")
        np.testing.assert_allclose(log.time, [0, .02, .04])
        np.testing.assert_allclose(log.column("ROLL_IMU(deg)"), [1, 2, 3])
        self.assertAlmostEqual(log.sample_rate, 50)
        self.assertTrue(any("重复" in note for note in log.warnings))
        self.assertFalse(log.data.flags.writeable)

    def test_explicit_seconds_override_large_increment_heuristic(self):
        log = self.log("TimeStamp(sec),value\n100,1\n102,2\n104,3\n")
        np.testing.assert_allclose(log.time, [0, 2, 4])
        self.assertEqual(log.time_unit, "s")
        forced = load_log(log.path, time_unit="ms")
        np.testing.assert_allclose(forced.time, [0, .002, .004])

    def test_rollover_and_reboot_are_distinguished(self):
        log = self.log("TimeStamp(us),value\n4294947296,1\n0,2\n20000,3\n")
        np.testing.assert_allclose(log.time, [0, .02, .04])
        with self.assertRaisesRegex(LogError, "倒退"):
            self.log("TimeStamp(us),value\n500000,1\n520000,2\n10000,3\n")

    def test_missing_timestamp_drops_matching_row_and_no_time_uses_fallback(self):
        log = self.log("Time(ms),value\n1000,1\n,9\n1040,3\n1060,4\n")
        np.testing.assert_allclose(log.column("value"), [1, 3, 4])
        log = self.log("value,other\n1,2\n3,4\n", fallback_hz=100)
        np.testing.assert_allclose(log.time, [0, .01])
        self.assertEqual(log.time_source, "Sample index")

    def test_empty_short_and_bad_data(self):
        for content in ("", "time(s),value\n", "time(s),value\n0,1\n"):
            with self.subTest(content=content), self.assertRaises(LogError):
                self.log(content)
        with self.assertRaises(LogError):
            self.log("Time(ms),value\n0,1\n20,nope\n40,3\n")
        log = self.log("Time(ms),value\n0,1\n20,nope\n40,3\n", skip_bad_rows=True)
        self.assertEqual(len(log.time), 2)
        self.assertTrue(any("跳过 1" in note for note in log.warnings))

    def test_column_mismatch_and_chunk_boundary_extra_fields(self):
        with self.assertRaises(LogError):
            self.log("time(s),a,b\n0,1\n1,2\n")
        with self.assertRaises(LogError):
            self.log("time(s),a\n0,1\n1,2\n2,3,4\n3,5\n", chunk_rows=2)

    def test_duplicate_headers_and_tab_delimiter(self):
        log = self.log("time(ms)\tx\tx\tx__2\n0\t1\t2\t3\n20\t4\t5\t6\n")
        self.assertEqual(len(set(log.columns)), 4)
        np.testing.assert_allclose(log.time, [0, .02])

    def test_modes_nan_is_unknown_and_air_detection_uses_current_names(self):
        time = np.arange(300) * .02
        speed = np.where((time > 1) & (time < 5), 15, 0)
        modes = np.where(time < 2, 1700, np.where(time < 4, 1500, 1300)).astype(float)
        modes[150] = np.nan
        rows = "\n".join(f"{t},{s},{m},1500" for t, s, m in zip(time, speed, modes))
        log = self.log("Time(s),airspeed_A,CH5_PWM,thro_PWM\n" + rows)
        self.assertEqual(log.in_air_source, "airspeed_A")
        self.assertEqual(len(log.in_air), 1)
        self.assertIn("Unknown", [segment.label for segment in log.modes])

    def test_fft_sine_amplitude_even_and_odd_lengths(self):
        for count in (1000, 1001):
            time = np.arange(count) / 100
            frequency = 50 * 100 / count
            result = compute_spectrum(time, 3 * np.sin(2 * np.pi * frequency * time) + 42)
            self.assertAlmostEqual(result.peak_frequency, frequency, places=6)
            self.assertAlmostEqual(result.peak_amplitude, 3, places=3)
            self.assertFalse(result.resampled)

    def test_fft_jitter_resampling_small_missing_value_and_large_gap(self):
        time = np.arange(1000) / 100
        time[1:-1] += .0004 * np.sin(np.arange(998))
        values = 2 * np.sin(2 * np.pi * 5 * time)
        values[200] = np.nan
        result = compute_spectrum(time, values)
        self.assertTrue(result.resampled)
        self.assertAlmostEqual(result.peak_frequency, 5, delta=.1)
        time[500:] += 3
        with self.assertRaisesRegex(LogError, "缺口"):
            compute_spectrum(time, values)
        with self.assertRaises(LogError):
            compute_spectrum(time, values, (1, 1.01))

    def test_decimation_preserves_impulse_and_nan_gap(self):
        time = np.arange(100000)
        values = np.zeros(100000)
        values[31415], values[50001], values[72000] = 99, -55, np.nan
        x, y = display_series(time, values, max_points=6000)
        self.assertLess(len(x), 6100)
        self.assertIn(99, y)
        self.assertIn(-55, y)
        self.assertTrue(np.isnan(y).any())
        self.assertTrue(np.all(np.diff(x) > 0))
        x, y = display_series([], [], smooth_samples=10)
        self.assertEqual(len(x), 0)

    def test_export_exact_raw_values_and_range(self):
        log = self.log("Time(ms),value\n0,1.123456789123\n20,2\n40,3\n")
        target = self.folder / "out.csv"
        export_csv(log, target, ["value"], (.02, .04))
        exported = load_log(target, time_column="Time_elapsed(s)")
        np.testing.assert_allclose(exported.column("value"), [2, 3])
        self.assertEqual(exported.data.shape[1], 2)


if __name__ == "__main__":
    unittest.main()
