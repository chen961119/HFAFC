import math
import unittest

from station_protocol import decode_fields
from test_station import CONFIG
from wing_geometry import wing_segments, configuration_ready, ORDERS


class WingGeometryTests(unittest.TestCase):
    def configuration(self, count=3, **values):
        result = decode_fields("CONFIG", CONFIG.split("\t"))
        result.update(count=count, roll_a=0, ab=0, ac=0, bd=0, ce=0, df=0, eg=0)
        result.update(values)
        return result

    def test_flat_chain_and_shared_tips_for_each_supported_count(self):
        for count, order in ORDERS.items():
            chain = wing_segments(self.configuration(count))
            self.assertEqual(tuple(s[0] for s in chain), order)
            self.assertAlmostEqual(chain[-1][2][0]-chain[0][1][0], count)
            for (_, left, right, _), following in zip(chain, chain[1:]):
                self.assertEqual(right, following[1])
                self.assertEqual(left[1], 0)

    def test_relative_rolls_accumulate_outward_from_a(self):
        chain = wing_segments(self.configuration(7, roll_a=10, ab=20, bd=30, df=40,
                                                 ac=-20, ce=-30, eg=-40))
        self.assertEqual([s[3] for s in chain], [100, 60, 30, 10, -10, -40, -80])
        for current, following in zip(chain, chain[1:]):
            self.assertEqual(current[2], following[1])
        for _, left, right, _ in chain:
            self.assertAlmostEqual(math.dist(left, right), 1)

    def test_rear_view_positive_roll_lowers_right_tip(self):
        _, left, right, _ = wing_segments(self.configuration(1, roll_a=30))[0]
        self.assertGreater(right[1], left[1])
        self.assertGreater(right[0], left[0])

    def test_measurement_only_mt_cannot_change_geometry(self):
        state = self.configuration(3, ab=20)
        before = wing_segments(state)
        state.update(mt_angle=179, angle=179)
        self.assertEqual(wing_segments(state), before)

    def test_invalid_configuration_and_freshness(self):
        self.assertFalse(configuration_ready(self.configuration(3, left_valid=0)))
        self.assertFalse(configuration_ready(self.configuration(3, master=0)))
        self.assertTrue(configuration_ready(self.configuration(1, left_valid=0, right_valid=0)))
        for body in (CONFIG.replace("ab=-20", "ab=nan"), CONFIG.replace("count=3", "count=6"), CONFIG + "\tab=5"):
            with self.assertRaises(ValueError):
                decode_fields("CONFIG", body.split("\t"))
