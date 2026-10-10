"""USB transport integration against a localhost controller, no board required."""
import socket
import tempfile
import threading
import time
import tkinter as tk
import unittest
from pathlib import Path

from main import GroundStation
from protocol import ReplyFramer
from station_protocol import decode_fields

CAPS = "version=1\timu=BMI088_SPI\texternal=DISABLED\tairspeed=MS4525\trotate=MT6701\tbarometer=DISABLED\tradio=SBUS\tcalibration=STUB\tsensor_setting=STUB\tradio_calibration=STUB\tflight_modes=STUB\treboot=SUPPORTED"
DATA = "ms=1000\troll=12.5\tpitch=-4\tyaw=32\tgx=1\tgy=2\tgz=3\tax=0\tay=0\taz=1\tairspeed=5.5\tangle=7\tangle_valid=1\trc1=1400\trc2=1500\trc3=1100\trc4=1600\trc5=1000\trc6=2000\tmode=MANUAL\tlocked=1"
CONFIG = "count=3\tmaster=1\troll_a=12.5\tab=-20\tac=30\tbd=0\tce=0\tdf=0\teg=0\tleft_valid=1\tright_valid=1"


class Controller:
    def __init__(self):
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen()
        self.listener.settimeout(.1)
        self.url = f"socket://127.0.0.1:{self.listener.getsockname()[1]}"
        self.stop = threading.Event()
        self.commands = []
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        connection = None
        try:
            while not self.stop.is_set():
                try:
                    connection, _ = self.listener.accept()
                    break
                except socket.timeout:
                    continue
            if connection is None:
                return
            connection.settimeout(.1)
            buffer = b""
            while not self.stop.is_set():
                try:
                    chunk = connection.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    return
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    command = line.decode("ascii").split()
                    self.commands.append(command)
                    prefix, verb, request_id = command[:3]
                    if prefix == "GCS":
                        body = {"CAPS": "CAPS\t" + CAPS, "DATA": "DATA\t" + DATA,
                                "CONFIG": "CONFIG\t" + CONFIG, "REBOOT": "REBOOT\tstatus=REBOOTING"}.get(verb, "ERROR\tNOT_IMPLEMENTED\tReserved for V2")
                        reply = f"@COFLY\t{request_id}\t{body}\n"
                    else:
                        reply = (f"@COFLY\t{request_id}\tBEGIN\tA-TEAM-3\t1\tSD_READY\tLOADED\n"
                                 f"@COFLY\t{request_id}\tVALUE\tKp_roll_angle\tfloat\t0.25\t0\t10\tAttitude\tRoll P\n"
                                 f"@COFLY\t{request_id}\tEND\t1\n")
                    raw = ("board log\n" + reply).encode()
                    for index in range(0, len(raw), 11):
                        connection.sendall(raw[index:index+11])
        except OSError:
            pass
        finally:
            if connection:
                connection.close()

    def close(self):
        self.stop.set()
        self.listener.close()
        self.thread.join(timeout=1)


class StationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = GroundStation(self.root, Path(self.directory.name) / "settings.json")
        self.controller = Controller()
        self.app.port.set(self.controller.url)
        self.app.toggle_connection()

    def tearDown(self):
        self.app.close()
        self.controller.close()
        self.directory.cleanup()

    def wait_for(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.root.update()
            if predicate():
                return
            time.sleep(.01)
        self.fail("Timed out waiting for station condition")

    def test_fragmented_handshake_and_live_data(self):
        self.wait_for(lambda: self.app.last_data > 0)
        self.assertEqual(self.app.caps["imu"], "BMI088_SPI")
        self.assertEqual(self.app.metric_vars["airspeed"].get(), "5.50")
        self.assertEqual(self.app.metric_vars["groundspeed"].get(), "—")
        self.assertEqual(self.app.metric_vars["rc3"].get(), "1100 μs")
        self.assertEqual(len(self.app.sensor_tree.get_children()), 5)

    def test_reboot_button_and_disconnect_after_ack(self):
        self.app.show_page("通讯连接")
        self.wait_for(lambda: bool(self.app.caps) and self.app.gcs_pending is None)
        self.assertEqual(str(self.app.reboot_button["state"]), "normal")
        self.app.reboot_button.invoke()
        self.wait_for(lambda: not self.app.connected)
        self.assertTrue(any(c[0:2] == ["GCS", "REBOOT"] for c in self.controller.commands))
        self.assertIn("已接受重启", self.app.status.get())
        self.assertEqual(str(self.app.reboot_button["state"]), "disabled")

    def test_live_configuration_and_missing_peer(self):
        self.wait_for(lambda: bool(self.app.configuration))
        self.assertEqual(len(self.app.wings.find_withtag("wing")), 3)
        self.assertIn("B—A—C", self.app.configuration_label.get())
        self.app.configuration["left_valid"] = 0
        self.app._draw_configuration()
        self.assertEqual(len(self.app.wings.find_withtag("wing")), 0)
        self.app.disconnect()
        self.assertFalse(self.app.configuration)

    def test_parameter_page_and_reserved_operations_share_connection(self):
        self.wait_for(lambda: bool(self.app.caps))
        self.app.show_page("参数")
        self.wait_for(lambda: self.app.gcs_pending is None)
        self.app.read_parameters()
        self.wait_for(lambda: "Kp_roll_angle" in self.app.parameters)
        for verb, args in (("CALIBRATE", ("IMU",)), ("SENSOR_SET", ("IMU", "DEFAULT")),
                           ("RADIO_CALIBRATE", ()), ("FLIGHT_MODES", ("MANUAL", "STABILIZE_NO_I", "STABILIZE"))):
            self.app._request(verb, *args)
            self.wait_for(lambda: self.app.gcs_pending is None)
            self.assertIn("暂未实现", self.app.status.get())
        self.assertEqual(self.app.parameters["Kp_roll_angle"].value, .25)

    def test_stale_disconnect_and_delayed_reply(self):
        self.wait_for(lambda: bool(self.app.data))
        self.app.show_page("参数")
        self.wait_for(lambda: self.app.gcs_pending is None)
        self.app.last_data = time.monotonic() - 3
        self.wait_for(lambda: not self.app.data)
        self.assertEqual(self.app.metric_vars["airspeed"].get(), "—")
        self.app.disconnect()
        self.app._handle_reply((99, "CAPS", CAPS.split("\t")))
        self.assertFalse(self.app.caps)
        self.assertEqual(self.app.sensor_tree.get_children(), ())
        self.assertTrue(self.app.auto_probe)

    def test_settings_and_all_pages(self):
        for name in self.app.pages:
            self.app.show_page(name)
            self.root.update()
        self.app.theme.set("室外")
        self.app.font_size.set(13)
        self.app.apply_appearance()
        saved = self.app.settings_path.read_text(encoding="utf-8")
        self.assertIn('"font_size": 13', saved)
        self.assertIn("室外", saved)

    def test_capability_and_data_validation(self):
        config = "count=3\tmaster=1\troll_a=0\tab=0\tac=0\tbd=0\tce=0\tdf=0\teg=0\tleft_valid=1\tright_valid=1"
        values = decode_fields("CONFIG", (config + "\taircraft_id=1\tconfig_valid=1").split("\t"))
        self.assertEqual(values["aircraft_id"], 1)
        for suffix in ("\taircraft_id=0\tconfig_valid=1", "\taircraft_id=4\tconfig_valid=1", "\taircraft_id=1", "\taircraft_id=nan\tconfig_valid=1"):
            with self.assertRaises(ValueError):
                decode_fields("CONFIG", (config + suffix).split("\t"))
        for body in (DATA.replace("gx=1", "gx=nan"), DATA + "\tgx=4", DATA.replace("locked=1", "locked=4")):
            with self.assertRaises(ValueError):
                decode_fields("DATA", body.split("\t"))
        with self.assertRaises(ValueError):
            decode_fields("CAPS", CAPS.replace("version=1", "version=2").split("\t"))
        framer = ReplyFramer()
        self.assertEqual(framer.feed(b"log\n@COFLY\t7\tCAP"), [])
        replies = framer.feed(("S\t" + CAPS + "\n").encode())
        self.assertEqual(decode_fields(replies[0][1], replies[0][2])["version"], "1")
