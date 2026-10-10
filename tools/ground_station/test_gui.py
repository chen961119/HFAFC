"""GUI/serial integration using a localhost mock device; no flight hardware."""
import socket
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch, Mock
from collections import deque
from pathlib import Path
import tempfile
from main import ParameterConsole
from protocol import Parameter
from parameter_file import encode_sd_file


class MockFlightController:
    def __init__(self):
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen()
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.value = "0.25"
        self.commands = []
        self.stopping = threading.Event()
        self.connection = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stopping.is_set():
            try:
                self.connection, _ = self.listener.accept()
                break
            except socket.timeout:
                continue
            except OSError:
                return
        if self.connection is None:
            return
        self.connection.settimeout(0.1)
        buffer = bytearray()
        try:
            while not self.stopping.is_set():
                try:
                    chunk = self.connection.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    return
                buffer.extend(chunk)
                while b"\n" in buffer:
                    line, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    self.commands.append(line.decode())
                    parts = line.decode().split()
                    if parts[:2] == ["PARAM", "READ"]:
                        request_id = parts[2]
                        reply = (f"普通日志：1111\n@COFLY\t{request_id}\tBEGIN\tF-TEAM-7-INDI-EXP\t1\tSD_READY\tLOADED\n"
                                 f"@COFLY\t{request_id}\tVALUE\tKp_roll_angle\t{self.value}\t0\t10\tAttitude\tRoll P\n"
                                 f"@COFLY\t{request_id}\tEND\t1\n").encode()
                        # Split inside UTF-8 characters and parameter fields.
                        for i in range(0, len(reply), 7):
                            self.connection.sendall(reply[i:i + 7])
                    elif parts[:2] == ["PARAM", "SET"]:
                        if abs(float(parts[4]) - 0.9) < 1e-6:
                            reply = f"@COFLY\t{parts[2]}\tERROR\tSD_WRITE\tRAM unchanged\n"
                        elif abs(float(parts[4]) - 0.8) < 1e-6:
                            continue  # Lost confirmation: exercise timeout UI.
                        else:
                            self.value = parts[4]
                            reply = f"@COFLY\t{parts[2]}\tOK\tKp_roll_angle\t{self.value}\tSAVED\n"
                        self.connection.sendall(reply.encode())
                    else:
                        self.connection.sendall(b"manual command received\n")
        except OSError:
            pass

    def close(self):
        self.stopping.set()
        if self.connection:
            self.connection.close()
        self.listener.close()
        self.thread.join(timeout=1)


class GuiTests(unittest.TestCase):
    def test_edited_file_requires_confirmation_before_usb_write(self):
        self.app.connected = True
        self.app.worker = Mock()
        self.app.parameters = {"gain": Parameter("gain", .25, 0, 10, "T", "T")}
        self.app.parameter_profile = "BOARD-1234567887654321-TEAM-INDI-EXP"
        data = encode_sd_file(self.app.parameters, self.app.parameter_profile)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.cfg"
            path.write_bytes(data.replace(b"0.25", b"0.5"))
            with patch("parameter_view.filedialog.askopenfilename", return_value=str(path)), \
                    patch("parameter_view.messagebox.askyesno", return_value=False) as confirm:
                self.app.load_parameters_from_file()
                confirm.assert_called_once()
            self.app.worker.send.assert_not_called()
            self.assertIsNone(self.app.file_import)
            self.assertIn("已取消", self.app.status.get())
            with patch("parameter_view.filedialog.askopenfilename", return_value=str(path)), \
                    patch("parameter_view.messagebox.askyesno", return_value=True) as confirm:
                self.app.load_parameters_from_file()
                confirm.assert_called_once()
            self.assertEqual(self.app.pending["name"], "gain")
            self.assertIn(b"gain 0.5", self.app.worker.send.call_args.args[0])
            self.assertEqual(path.read_bytes(), data.replace(b"0.25", b"0.5"))

    def test_invalid_edited_file_cannot_be_confirmed_or_sent(self):
        self.app.connected = True
        self.app.worker = Mock()
        self.app.parameters = {"gain": Parameter("gain", .25, 0, 10, "T", "T")}
        self.app.parameter_profile = "BOARD-1234567887654321-TEAM-INDI-EXP"
        data = encode_sd_file(self.app.parameters, self.app.parameter_profile)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.cfg"
            path.write_bytes(data.replace(b"0.25", b"11"))
            with patch("parameter_view.filedialog.askopenfilename", return_value=str(path)), \
                    patch("parameter_view.messagebox.askyesno", return_value=True) as confirm:
                self.app.load_parameters_from_file()
                confirm.assert_not_called()
            self.app.worker.send.assert_not_called()
            self.assertIn("文件导入失败", self.app.status.get())

    def test_file_import_stops_after_rejection_keeps_confirmed_ram_change(self):
        self.app.connected = True
        self.app.worker = Mock()
        self.app.parameters = {name: Parameter(name, .25, 0, 10, "T", "T") for name in ("first", "second", "third")}
        self.app.file_import = {"queue": deque([("second", 2), ("third", 3)]), "done": 0, "ram_only": 0, "acked": False}
        self.app.pending = {"id": 9, "kind": "SET", "name": "first"}
        self.app._handle_reply((9, "OK", ["first", "float", "1", "RAM_ONLY", "3"]))
        self.assertEqual(self.app.pending["name"], "second")
        self.app._handle_reply((self.app.pending["id"], "ERROR", ["LOCK_REQUIRED", "Lock first"]))
        self.assertIsNone(self.app.file_import)
        self.assertEqual(self.app.parameters["first"].value, 1)
        self.assertEqual(self.app.parameters["third"].value, .25)
        self.assertIn("已确认应用 1 项", self.app.status.get())
        self.assertEqual(self.app.worker.send.call_count, 1)

    def test_ram_only_ack_updates_table_without_claiming_saved(self):
        self.app.parameters = {"gain": Parameter("gain", .25, 0, 10, "T", "T")}
        self.app.pending = {"id": 9, "kind": "SET", "name": "gain"}
        self.app._handle_reply((9, "OK", ["gain", "float", "2", "RAM_ONLY", "7"]))
        self.assertEqual(self.app.parameters["gain"].value, 2)
        self.assertEqual(self.app.parameter_generation, 7)
        self.assertIn("未持久保存", self.app.status.get())

    def test_local_file_save_and_serial_import_exclude_eeprom(self):
        self.app.port.set(f"socket://127.0.0.1:{self.device.port}")
        self.app.toggle_connection()
        self.wait_for(lambda: self.app.connected)
        self.app.parameters = {
            "Kp_roll_angle": Parameter("Kp_roll_angle", .35, 0, 10, "T", "T"),
            "aircraft_id": Parameter("aircraft_id", 1, 0, 7, "A", "A", "int", "EEPROM", "REBOOT"),
        }
        self.app.parameter_profile = "BOARD-1234567887654321-TEAM-INDI-EXP"
        self.app.parameter_generation = 3
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.cfg"
            with patch("parameter_view.filedialog.asksaveasfilename", return_value=str(path)):
                self.app.save_parameters_to_file()
            self.assertNotIn(b"aircraft_id", path.read_bytes())
            with patch("parameter_view.filedialog.askopenfilename", return_value=str(path)):
                self.app.load_parameters_from_file()
            self.wait_for(lambda: self.app.file_import is None)
            self.assertIn("文件导入完成", self.app.status.get())
            self.assertTrue(any("SET" in c and "Kp_roll_angle" in c for c in self.device.commands))
            self.assertFalse(any("aircraft_id" in c for c in self.device.commands))
            self.assertEqual(self.app.parameters["aircraft_id"].value, 1)

    def test_eeprom_reboot_ack_and_read_only_editor(self):
        self.app.connected = True
        self.app.parameters = {
            "aircraft_id": Parameter("aircraft_id", 1, 0, 7, "Aircraft", "Identity", "int", "EEPROM", "REBOOT"),
            "valid": Parameter("valid", 0, 0, 1, "IMU", "Valid", "int", "EEPROM", "IMMEDIATE", True),
        }
        self.app._render_table()
        self.app.pending = {"id": 9, "kind": "SET", "name": "aircraft_id"}
        self.app._handle_reply((9, "OK", ["aircraft_id", "int", "2", "SAVED"]))
        self.assertIn("EEPROM", self.app.status.get())
        self.assertIn("重启后生效", self.app.status.get())
        self.assertEqual(self.app.tree.set("aircraft_id", "storage"), "EEPROM")
        self.app._begin_cell_edit("valid")
        self.assertIsNone(self.app.edit_box)

    def setUp(self):
        self.device = MockFlightController()
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = ParameterConsole(self.root)

    def tearDown(self):
        self.app.close()
        self.device.close()

    def wait_for(self, condition, seconds=4):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.root.update()
            if condition():
                return
            time.sleep(0.01)
        self.fail("Timed out waiting for GUI/serial event")

    def test_read_write_terminal_failure_timeout_reconnect(self):
        self.app.port.set(f"socket://127.0.0.1:{self.device.port}")
        self.app.toggle_connection()
        self.wait_for(lambda: self.app.connected)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "raw.log"
            with open(path, "ab", buffering=0) as log:
                self.app.log_file = log
                try:
                    self.app.read_parameters()
                    self.wait_for(lambda: self.app.pending is None and bool(self.app.parameters))
                    self.wait_for(lambda: "普通日志：1111" in self.app.terminal.get("1.0", "end"))
                    self.assertEqual(self.app.parameters["Kp_roll_angle"].value, 0.25)
                    self.assertIn("普通日志：1111", self.app.terminal.get("1.0", "end"))
                    self.assertIn("普通日志：1111".encode(), path.read_bytes())
                finally:
                    self.app.log_file = None
        self.app.tree.selection_set("Kp_roll_angle")
        self.root.update()
        self.app.new_value.set("0.75")
        self.app.write_parameter()
        self.wait_for(lambda: self.app.pending is None)
        self.assertEqual(self.device.value, "0.75")
        self.assertEqual(self.app.parameters["Kp_roll_angle"].value, 0.75)
        self.assertIn("已保存 SD", self.app.status.get())

        self.app.new_value.set("0.9")
        self.app.write_parameter()
        self.wait_for(lambda: self.app.pending is None)
        self.assertEqual(self.app.parameters["Kp_roll_angle"].value, 0.75)
        self.assertIn("内存未修改", self.app.status.get())
        self.app.new_value.set("0.8")
        self.app.write_parameter()
        self.wait_for(lambda: self.app.pending is not None)
        self.app.pending["deadline"] = time.monotonic() - 1
        self.wait_for(lambda: self.app.pending is None)
        self.assertIn("写入结果未确认", self.app.status.get())

        self.app.command.set("custom command")
        self.app.send_command()
        self.wait_for(lambda: "manual command received" in self.app.terminal.get("1.0", "end"))
        self.assertIn("custom command", self.device.commands)
        self.app.filter_text.set("not-matching")
        self.assertEqual(len(self.app.tree.get_children()), 0)
        self.app.filter_text.set("")
        self.assertEqual(len(self.app.tree.get_children()), 1)
        old_id = self.app.connection_id
        self.app.disconnect()
        self.app.events.put((old_id, "rx", b"@COFLY\t1\tEND\t1\n"))
        self.root.update()
        self.assertFalse(self.app.parameters)
        self.assertFalse(self.app.connected)

    def test_incomplete_snapshot_never_replaces_existing_parameters(self):
        self.app.pending = {"id": 5, "kind": "READ", "count": None}
        self.app._handle_reply((5, "BEGIN", ["F", "2", "SD_READY", "LOADED"]))
        self.app._handle_reply((5, "VALUE", ["p", "0.25", "0", "10", "g", "desc"]))
        self.app._handle_reply((5, "END", ["2"]))
        self.assertFalse(self.app.parameters)
        self.assertIn("不完整", self.app.status.get())

    def test_migrated_snapshot_shows_recovery_counts(self):
        self.app.pending = {"id": 6, "kind": "READ", "count": None}
        self.app._handle_reply((6, "BEGIN", ["F", "2", "SD_READY", "MIGRATED", "1", "1", "1"]))
        self.app._handle_reply((6, "VALUE", ["retained", "float", "0.25", "0", "10", "g", "desc"]))
        self.app._handle_reply((6, "VALUE", ["added", "int", "2", "0", "10", "g", "desc"]))
        self.app._handle_reply((6, "END", ["2"]))
        self.assertEqual(len(self.app.parameters), 2)
        self.assertIn("已迁移到新参数表", self.app.status.get())
        self.assertIn("开机匹配 1 / 默认 1 / 跳过旧项 1", self.app.status.get())

    def test_complete_75_and_512_parameter_snapshots(self):
        for count in (75, 512):
            with self.subTest(count=count):
                self.app.pending = {"id": count, "kind": "READ", "count": None}
                self.app._handle_reply((count, "BEGIN", ["F", str(count), "SD_READY", "LOADED"]))
                self.assertIsNotNone(self.app.pending)
                for index in range(count):
                    self.app._handle_reply((count, "VALUE", [f"p{index}", "float", "0.2", "0", "10", "Ground", "Tuning value"]))
                self.app._handle_reply((count, "END", [str(count)]))
                self.assertIsNone(self.app.pending)
                self.assertEqual(len(self.app.parameters), count)
                self.assertEqual(len(self.app.tree.get_children()), count)
                self.assertEqual(self.app.tree.set(f"p{count - 1}", "value"), "0.2")
                self.assertIn(f"已读取 {count} 项", self.app.status.get())
        previous = dict(self.app.parameters)
        for count in (0, 513):
            self.app.pending = {"id": 900, "kind": "READ", "count": None}
            self.app._handle_reply((900, "BEGIN", ["F", str(count), "SD_READY", "LOADED"]))
            self.assertIsNone(self.app.pending)
            self.assertIn("参数数量无效", self.app.status.get())
            self.assertEqual(self.app.parameters, previous)

    def test_sort_float_display_and_integer_ack(self):
        self.app.parameters = {
            "p10": Parameter("p10", 0.200000003, 0, 10, "g", "ten"),
            "p2": Parameter("p2", 1, 0, 1, "g", "switch", "int"),
        }
        self.app._render_table()
        self.assertEqual(self.app.tree.set("p10", "value"), "0.2")
        self.assertEqual(self.app.tree.set("p2", "dtype"), "int")
        self.app._sort_heading("name")
        self.assertEqual(self.app.tree.get_children(), ("p2", "p10"))
        self.app._sort_heading("name")
        self.assertEqual(self.app.tree.get_children(), ("p10", "p2"))
        self.app._sort_heading("value")
        self.assertEqual(self.app.tree.get_children(), ("p10", "p2"))
        self.app.pending = {"id": 9, "kind": "SET", "name": "p2"}
        self.app._handle_reply((9, "OK", ["p2", "int", "0", "SAVED"]))
        self.assertEqual(self.app.parameters["p2"].value, 0)
        self.assertIn("已保存", self.app.status.get())

    def test_hex_switch_preserves_raw_bytes_and_text_history(self):
        wire = "中文日志\n".encode() + bytes([0, 255, 10])
        self.app.events.put((self.app.connection_id, "rx", wire))
        self.wait_for(lambda: "中文日志" in self.app.terminal.get("1.0", "end"))
        self.app.hex_display.set(True)
        self.app._rebuild_terminal()
        self.assertIn("00 FF 0A", self.app.terminal.get("1.0", "end"))
        self.app.hex_display.set(False)
        self.app._rebuild_terminal()
        self.assertIn("中文日志", self.app.terminal.get("1.0", "end"))
        self.app._clear_terminal()
        self.assertFalse(self.app.history)

    def test_small_window_keeps_write_and_send_controls_visible(self):
        self.root.deiconify()
        for size in ("760x520", "900x620", "1160x780", "760x520"):
            self.root.geometry(size)
            self.root.update()
            for widget in (self.app.tree,
                           self.app.command_box, self.app.send_button):
                self.assertTrue(widget.winfo_ismapped())
                self.assertGreater(widget.winfo_height(), 5)
                self.assertLessEqual(widget.winfo_rooty() + widget.winfo_height(),
                                     self.root.winfo_rooty() + self.root.winfo_height())
                self.assertLessEqual(widget.winfo_rootx() + widget.winfo_width(),
                                     self.root.winfo_rootx() + self.root.winfo_width())

    def test_inline_edit_selection_cancel_and_save(self):
        self.root.deiconify()
        self.app.port.set(f"socket://127.0.0.1:{self.device.port}")
        self.app.toggle_connection()
        self.wait_for(lambda: self.app.connected)
        self.app.read_parameters()
        self.wait_for(lambda: self.app.pending is None and bool(self.app.parameters))
        self.app._begin_cell_edit("Kp_roll_angle")
        self.app.edit_box.focus_force()
        self.root.update()
        self.assertIsNotNone(self.app.edit_box)
        self.assertEqual(self.app.edit_box.master, self.app.tree)
        self.assertTrue(self.app.edit_box.selection_present())
        self.app.new_value.set("0.7")
        self.app.edit_box.event_generate("<Escape>")
        self.root.update()
        self.assertIsNone(self.app.edit_box)
        self.assertEqual(self.device.value, "0.25")
        self.app._begin_cell_edit("Kp_roll_angle")
        self.app.edit_box.focus_force()
        self.root.update()
        self.app.new_value.set("0.75")
        self.app.edit_box.event_generate("<Return>")
        self.wait_for(lambda: self.app.pending is None and self.device.value == "0.75")
        self.assertIsNone(self.app.edit_box)
        self.assertEqual(self.app.tree.set("Kp_roll_angle", "value"), "0.75")


if __name__ == "__main__":
    unittest.main()
