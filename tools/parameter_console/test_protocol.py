import unittest
from protocol import Parameter, ReplyFramer, parameter_from_fields, validate_edit, format_value
import struct


class ProtocolTests(unittest.TestCase):
    def test_short_float_roundtrip_and_integer_type(self):
        self.assertEqual(format_value(0.200000003), "0.2")
        self.assertEqual(format_value(0.00001), "0.00001")
        self.assertEqual(format_value(0.0002), "0.0002")
        self.assertEqual(format_value(1e20), "100000000000000000000")
        for bits in (1, 0x00800000, 0x7f7fffff, 0x80000001, 0xff7fffff):
            value = struct.unpack("<f", struct.pack("<I", bits))[0]
            text = format_value(value)
            self.assertNotIn("e", text.lower())
            self.assertEqual(struct.pack("<f", float(text)), struct.pack("<f", value))
        for value in (0.11, 0.123456789, -113.65, 1e-25, -0.0):
            self.assertEqual(struct.pack("<f", value), struct.pack("<f", float(format_value(value))))
        p = parameter_from_fields(["flag", "int", "0", "-2147483648", "2147483647", "g", "d"])
        self.assertEqual(validate_edit(p, "2147483647"), 2147483647)
        self.assertEqual(validate_edit(p, "-2147483648"), -2147483648)
        for text in ("0.5", "1.0", "1e0", "2147483648", "nan"):
            with self.assertRaises(ValueError):
                validate_edit(p, text)
        direction = Parameter("pwm_channel3_rev", 1, -1, 1, "Actuator", "Direction", "int")
        self.assertEqual(validate_edit(direction, "-1"), -1)
        with self.assertRaises(ValueError):
            validate_edit(direction, "0")
    def test_split_replies_mixed_with_logs(self):
        wire = ("1111\n调试日志\n@COFLY\t3\tBEGIN\tF-TEAM-7-INDI-EXP\t1\tSD_READY\tLOADED\r\n"
                "@COFLY\t3\tVALUE\tKp_roll_angle\t0.25\t0\t10\tAttitude\tRoll P\n"
                "noise\n@COFLY\t3\tEND\t1\n").encode()
        for size in (1, 2, 7, 256, 4096):
            framer = ReplyFramer()
            replies = []
            for i in range(0, len(wire), size):
                replies.extend(framer.feed(wire[i:i + size]))
            self.assertEqual([r[1] for r in replies], ["BEGIN", "VALUE", "END"])
            self.assertEqual(parameter_from_fields(replies[1][2]).value, 0.25)

    def test_bad_frames_and_long_log_do_not_break_following_reply(self):
        framer = ReplyFramer()
        self.assertFalse(framer.feed(b"x" * 4096 + b"\n@COFLY\t4294967296\tEND\t1\n"))
        self.assertEqual(framer.feed(b"@COFLY\t4\tEND\t1\n"), [(4, "END", ["1"])])

    def test_parameter_validation(self):
        parameter = Parameter("gain", 0.25, 0, 10, "Attitude", "P")
        for text in ("nan", "inf", "-inf", "bad", "11", "-1", "1e1000"):
            with self.assertRaises(ValueError):
                validate_edit(parameter, text)
        self.assertEqual(validate_edit(parameter, "0.75"), 0.75)
        with self.assertRaises(ValueError):
            parameter_from_fields(["p", "nan", "0", "10", "g", "desc"])

    def test_float32_limits_accept_their_nominal_decimal_value(self):
        parameter = Parameter("effectiveness", -113.65, -10000, -0.00100000005, "INDI", "G")
        self.assertAlmostEqual(validate_edit(parameter, "-0.001"), -0.001, places=9)
        with self.assertRaises(ValueError):
            validate_edit(parameter, "-0.0001")


if __name__ == "__main__":
    unittest.main()
