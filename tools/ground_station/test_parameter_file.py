import unittest
import zlib

from parameter_file import CrcMismatchError, encode_sd_file, decode_sd_file
from protocol import Parameter

PROFILE = "BOARD-1234567887654321-TEAM-INDI-EXP"


class ParameterFileTests(unittest.TestCase):
    def setUp(self):
        self.parameters = {
            "gain": Parameter("gain", .25, 0, 10, "T", "T"),
            "pwm_rev": Parameter("pwm_rev", -1, -1, 1, "T", "T", "int"),
            "aircraft_id": Parameter("aircraft_id", 2, 0, 7, "A", "A", "int", "EEPROM", "REBOOT"),
        }

    def test_exact_sd_bytes_and_eeprom_exclusion(self):
        body = f"COFLY_PARAMS_V5 {PROFILE}\nGEN=12\nCOUNT=2\ngain:float=0.25\npwm_rev:int=-1\n".encode()
        data = encode_sd_file(self.parameters, PROFILE, 12)
        self.assertEqual(data, body + f"CRC32={zlib.crc32(body):08x}\n".encode())
        self.assertNotIn(b"aircraft_id", data)
        self.assertEqual(decode_sd_file(data, self.parameters, PROFILE), {"gain": .25, "pwm_rev": -1})

    def test_other_board_same_configuration_can_import_over_usb(self):
        data = encode_sd_file(self.parameters, PROFILE)
        self.assertEqual(decode_sd_file(data, self.parameters, PROFILE.replace("1234567887654321", "1111111122222222"))["gain"], .25)
        with self.assertRaises(ValueError):
            decode_sd_file(data, self.parameters, PROFILE.replace("INDI", "PID"))

    def corrupt_record(self, source, replacement):
        data = encode_sd_file(self.parameters, PROFILE)
        body = data[:data.rfind(b"CRC32=")].replace(source, replacement)
        return body + f"CRC32={zlib.crc32(body):08x}\n".encode()

    def test_bad_crc_truncation_and_crlf_rejected(self):
        data = encode_sd_file(self.parameters, PROFILE)
        for invalid in (data[:-1], data.replace(b"0.25", b"0.26"), data.replace(b"\n", b"\r\n"), b"\xef\xbb\xbf"+data):
            with self.assertRaises(ValueError):
                decode_sd_file(invalid, self.parameters, PROFILE)

    def test_duplicate_unknown_eeprom_type_range_and_missing_rejected(self):
        for source, replacement in ((b"pwm_rev:int=-1", b"gain:float=1"),
                                    (b"gain:float=0.25", b"unknown:float=0.25"),
                                    (b"gain:float=0.25", b"aircraft_id:int=1"),
                                    (b"gain:float=0.25", b"gain:int=1"),
                                    (b"gain:float=0.25", b"gain:float=11"),
                                    (b"COUNT=2\n", b"COUNT=1\n")):
            with self.assertRaises(ValueError):
                decode_sd_file(self.corrupt_record(source, replacement), self.parameters, PROFILE)

    def test_crc_override_only_relaxes_checksum(self):
        data = encode_sd_file(self.parameters, PROFILE)
        edited = data.replace(b"0.25", b"0.26")
        with self.assertRaises(CrcMismatchError):
            decode_sd_file(edited, self.parameters, PROFILE)
        self.assertAlmostEqual(decode_sd_file(edited, self.parameters, PROFILE, allow_crc_mismatch=True)["gain"], .26)
        for invalid in (data.replace(b"0.25", b"11"),
                        data.replace(b"gain:float", b"gain:int"),
                        data.replace(b"gain:float", b"unknown:float"),
                        data.replace(b"COUNT=2", b"COUNT=1"),
                        data[:data.rfind(b"CRC32=")],
                        data.replace(b"CRC32=", b"CRC32=x")):
            with self.assertRaises(ValueError):
                decode_sd_file(invalid, self.parameters, PROFILE, allow_crc_mismatch=True)

    def test_float32_limits_plain_decimal_and_negative_zero(self):
        self.parameters["gain"] = Parameter("gain", -0., -3.4028234663852886e38, 3.4028234663852886e38, "T", "T")
        data = encode_sd_file(self.parameters, PROFILE)
        self.assertIn(b"gain:float=-0\n", data)
        self.assertEqual(decode_sd_file(data, self.parameters, PROFILE)["gain"], -0.)

    def test_read_only_float_uses_float32_equality(self):
        self.parameters["gain"] = Parameter("gain", .3, 0, 10, "T", "T", read_only=True)
        data = encode_sd_file(self.parameters, PROFILE)
        self.assertEqual(decode_sd_file(data, self.parameters, PROFILE)["pwm_rev"], -1)
        with self.assertRaises(ValueError):
            decode_sd_file(self.corrupt_record(b"gain:float=0.3", b"gain:float=0.4"), self.parameters, PROFILE)


if __name__ == "__main__":
    unittest.main()
