"""USB roles must follow interface IDs, independent of enumeration/COM order."""
import unittest
from types import SimpleNamespace

from prepare import select_usb_ports, usb_interface


def port(device, interface, serial="board", location="1-2", pid=0x048B):
    return SimpleNamespace(device=device, vid=0x16C0, pid=pid, hwid="USB VID:PID=16C0:048B",
                           serial_number=serial, location=f"{location}:x.{interface}")


class UsbPortTests(unittest.TestCase):
    def test_com_order_does_not_choose_role(self):
        ground, debug = port("COM20", 0), port("COM3", 2)
        self.assertEqual(select_usb_ports([debug, ground]), (ground, debug))

    def test_linux_location(self):
        item = port("/dev/ttyACM1", 2)
        item.location = "1-2:1.2"
        self.assertEqual(usb_interface(item), 2)

    def test_raw_windows_hardware_id(self):
        item = port("COM3", 2)
        item.location = None
        item.hwid = r"USB\VID_16C0&PID_048B&MI_02\instance"
        self.assertEqual(usb_interface(item), 2)

    def test_old_single_port_and_missing_interface_wait(self):
        self.assertIsNone(select_usb_ports([port("COM3", 0, pid=0x0483)]))
        self.assertIsNone(select_usb_ports([port("COM3", 2)]))

    def test_unknown_metadata_never_guesses(self):
        items = [port("COM3", 0), port("COM4", 2)]
        for item in items:
            item.location = None
        self.assertIsNone(select_usb_ports(items))

    def test_multiple_boards_and_mixed_pairs_rejected(self):
        with self.assertRaises(RuntimeError):
            select_usb_ports([port("COM3", 0), port("COM4", 2), port("COM5", 0, serial="other")])
        with self.assertRaises(RuntimeError):
            select_usb_ports([port("COM3", 0), port("COM4", 2, serial="other")])
        with self.assertRaises(RuntimeError):
            select_usb_ports([port("COM3", 0), port("COM4", 2, location="1-3")])


if __name__ == "__main__":
    unittest.main()
