"""CoFly Autopilot USB parameter editor and serial terminal. Run: python main.py"""
import codecs
from collections import deque
import re
from dataclasses import replace
from pathlib import Path
import queue
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    from serial.tools import list_ports
    from serial_worker import SerialWorker
except ImportError as exc:
    raise SystemExit("缺少 pySerial。请运行：python -m pip install -r requirements.txt") from exc
from protocol import ReplyFramer, parameter_from_fields, validate_edit, format_value

MAX_PARAMETERS = 512


class ParameterConsole:
    def __init__(self, root):
        self.root = root
        root.title("CoFly Autopilot 参数配置与串口助手")
        root.geometry("1160x780")
        root.minsize(760, 520)
        self.events = queue.Queue()
        self.worker = None
        self.connection_id = 0
        self.connected = False
        self.request_id = 0
        self.pending = None
        self.parameters = {}
        self.received_parameters = {}
        self.framer = ReplyFramer()
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")
        self.log_file = None
        self.port = tk.StringVar()
        self.baud = tk.StringVar(value="921600")
        self.status = tk.StringVar(value="未连接")
        self.selected_name = tk.StringVar(value="未选择参数")
        self.new_value = tk.StringVar()
        self.edit_box = None
        self.editing_name = None
        self.command = tk.StringVar()
        self.hex_display = tk.BooleanVar(value=False)
        self.sort_column = None
        self.sort_reverse = False
        self.history = deque()
        self.history_size = 0
        self.terminal_chars = 0
        self.display_pending = []
        self.last_display = 0
        self.follow = tk.BooleanVar(value=True)
        self.filter_text = tk.StringVar()
        self._build_widgets()
        self.refresh_ports()
        self._update_controls()
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.poll_timer = root.after(30, self._poll)

    def _build_widgets(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", rowheight=27)
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        connection = ttk.Frame(outer)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=3)
        outer.rowconfigure(5, weight=2)
        connection.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(connection, text="串口").pack(side="left")
        self.port_box = ttk.Combobox(connection, textvariable=self.port, width=19)
        self.port_box.pack(side="left", padx=8)
        self.refresh_button = ttk.Button(connection, text="刷新", command=self.refresh_ports)
        self.refresh_button.pack(side="left")
        ttk.Label(connection, text="波特率").pack(side="left", padx=(18, 8))
        self.baud_box = ttk.Combobox(connection, textvariable=self.baud, width=10,
                                     values=(115200, 500000, 921600))
        self.baud_box.pack(side="left")
        self.connect_button = ttk.Button(connection, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=10)
        self.read_button = ttk.Button(connection, text="读取参数", command=self.read_parameters)
        self.read_button.pack(side="left")
        self.help_button = ttk.Button(connection, text="命令说明", command=self.send_help)
        self.help_button.pack(side="left", padx=8)

        ttk.Label(outer, textvariable=self.status, wraplength=710).grid(row=1, column=0, sticky="w", pady=(0, 8))
        parameter_area = ttk.Labelframe(outer, text="参数表", padding=8)
        parameter_area.grid(row=2, column=0, sticky="nsew")
        search = ttk.Frame(parameter_area)
        search.pack(fill="x", pady=(0, 6))
        ttk.Label(search, text="筛选").pack(side="left")
        ttk.Entry(search, textvariable=self.filter_text, width=30).pack(side="left", padx=8)
        ttk.Label(search, text="双击当前值编辑；Enter 保存到内存和 SD，Esc 取消").pack(side="left")
        self.filter_text.trace_add("write", lambda *_: self._render_table())
        grid = ttk.Frame(parameter_area)
        grid.pack(fill="both", expand=True)
        columns = ("group", "name", "dtype", "value", "bounds", "description")
        self.tree = ttk.Treeview(grid, columns=columns, show="headings", selectmode="browse", height=10)
        for column, label, width in zip(columns, ("分组", "参数名", "类型", "当前值", "允许范围", "说明"),
                                        (100, 210, 80, 100, 140, 300)):
            self.tree.heading(column, text=label, command=lambda c=column: self._sort_heading(c))
            self.tree.column(column, width=width, minwidth=70, stretch=column == "description")
        vertical = ttk.Scrollbar(grid, orient="vertical", command=lambda *args: self._scroll_table("y", *args))
        horizontal = ttk.Scrollbar(grid, orient="horizontal", command=lambda *args: self._scroll_table("x", *args))
        self.tree.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        grid.rowconfigure(0, weight=1)
        grid.columnconfigure(0, weight=1)
        self.tree.bind("<<TreeviewSelect>>", self._select_parameter)
        self.tree.bind("<Double-1>", self._edit_cell)
        self.tree.bind("<Configure>", lambda _: self._cancel_edit())
        self.tree.bind("<MouseWheel>", lambda _: self._cancel_edit())

        terminal_tools = ttk.Frame(outer)
        terminal_tools.grid(row=4, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(terminal_tools, text="清空", command=lambda: self._clear_terminal()).pack(side="left")
        ttk.Button(terminal_tools, text="导出窗口内容", command=self.export_terminal).pack(side="left", padx=8)
        self.record_button = ttk.Button(terminal_tools, text="记录全部接收", command=self.toggle_recording)
        self.record_button.pack(side="left")
        ttk.Checkbutton(terminal_tools, text="自动滚动", variable=self.follow).pack(side="left", padx=12)
        ttk.Checkbutton(terminal_tools, text="16 进制显示", variable=self.hex_display,
                        command=self._rebuild_terminal).pack(side="left")
        terminal_frame = ttk.Frame(outer)
        terminal_frame.grid(row=5, column=0, sticky="nsew")
        self.terminal = tk.Text(terminal_frame, width=1, height=8, wrap="none", state="disabled",
                                background="#18222e", foreground="#e7edf3",
                                insertbackground="white", font=("Consolas", 10))
        terminal_y = ttk.Scrollbar(terminal_frame, command=self.terminal.yview)
        terminal_x = ttk.Scrollbar(terminal_frame, orient="horizontal", command=self.terminal.xview)
        self.terminal.configure(yscrollcommand=terminal_y.set, xscrollcommand=terminal_x.set)
        self.terminal.grid(row=0, column=0, sticky="nsew")
        terminal_y.grid(row=0, column=1, sticky="ns")
        terminal_x.grid(row=1, column=0, sticky="ew")
        terminal_frame.rowconfigure(0, weight=1)
        terminal_frame.columnconfigure(0, weight=1)
        send_area = ttk.Frame(outer)
        send_area.grid(row=6, column=0, sticky="ew", pady=(8, 0))
        self.send_frame = send_area
        ttk.Label(send_area, text="发送").pack(side="left")
        self.command_box = ttk.Entry(send_area, textvariable=self.command)
        self.command_box.pack(side="left", fill="x", expand=True, padx=8)
        self.command_box.bind("<Return>", lambda _: self.send_command())
        self.send_button = ttk.Button(send_area, text="发送", command=self.send_command)
        self.send_button.pack(side="left", padx=(8, 0))

    def refresh_ports(self):
        try:
            ports = sorted(list_ports.comports())
            self.port_box["values"] = [port.device for port in ports]
            if not self.port.get() and ports:
                self.port.set(ports[0].device)
        except OSError as exc:
            self.status.set(f"枚举串口失败：{exc}")

    def toggle_connection(self):
        if self.worker:
            self.disconnect()
            return
        if not self.port.get().strip():
            self.status.set("请选择串口")
            return
        try:
            baud = int(self.baud.get())
            if baud <= 0:
                raise ValueError
        except ValueError:
            self.status.set("请输入有效波特率")
            return
        self.connection_id += 1
        self.framer = ReplyFramer()
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")
        self.parameters.clear()
        self._render_table()
        self.selected_name.set("未选择参数")
        self.new_value.set("")
        self.worker = SerialWorker(self.port.get().strip(), baud, self.events, self.connection_id)
        self.worker.start()
        self.status.set("正在连接…")
        self._update_controls()

    def disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker.join(timeout=0.75)
        self.worker = None
        self.connected = False
        self.pending = None
        self.connection_id += 1  # Reject all delayed events from the old worker.
        self.parameters.clear()
        self._render_table()
        self.selected_name.set("未选择参数")
        self.new_value.set("")
        self.status.set("已断开；重新连接后请读取参数")
        self._update_controls()

    def _update_controls(self):
        state = "normal" if self.connected else "disabled"
        self.connect_button.configure(text="断开" if self.worker else "连接")
        self.port_box.configure(state="disabled" if self.worker else "normal")
        self.baud_box.configure(state="disabled" if self.worker else "normal")
        self.refresh_button.configure(state="disabled" if self.worker else "normal")
        self.send_button.configure(state=state)
        self.command_box.configure(state=state)
        self.help_button.configure(state=state)
        idle = self.connected and self.pending is None
        self.read_button.configure(state="normal" if idle else "disabled")
        if self.edit_box is not None and not idle:
            self._cancel_edit()

    def _scroll_table(self, axis, *args):
        self._cancel_edit()
        getattr(self.tree, axis + "view")(*args)

    def _cancel_edit(self, _=None):
        if self.edit_box is not None:
            self.edit_box.destroy()
            self.edit_box = None
        self.editing_name = None

    def _edit_cell(self, event):
        if self.tree.identify_region(event.x, event.y) != "cell":
            return
        if self.tree.identify_column(event.x) == "#4":
            self._begin_cell_edit(self.tree.identify_row(event.y))
            return "break"

    def _begin_cell_edit(self, name):
        self._cancel_edit()
        if not self.connected or self.pending or name not in self.parameters:
            return
        bounds = self.tree.bbox(name, "value")
        if not bounds:
            return
        self.tree.selection_set(name)
        self.editing_name = name
        parameter = self.parameters[name]
        self.new_value.set(format_value(parameter.value, parameter.dtype))
        self.edit_box = ttk.Entry(self.tree, textvariable=self.new_value)
        x, y, width, height = bounds
        self.edit_box.place(x=x, y=y, width=width, height=height)
        self.edit_box.bind("<Return>", lambda _: self.write_parameter())
        self.edit_box.bind("<Escape>", self._cancel_edit)
        # Ignore a delayed FocusOut belonging to an editor already replaced.
        self.edit_box.bind("<FocusOut>", lambda event: self._cancel_edit()
                           if event.widget is self.edit_box else None)
        self.edit_box.focus_set()
        self.edit_box.selection_range(0, tk.END)
        self.edit_box.icursor(tk.END)

    def _start_request(self, kind, name=None, value=None):
        if not self.connected or self.pending:
            return
        self.request_id = self.request_id % 0xFFFFFFFF + 1
        request_id = self.request_id
        if kind == "READ":
            command = f"PARAM READ {request_id}\n"
        else:
            command = f"PARAM SET {request_id} {name} {format_value(value, self.parameters[name].dtype)}\n"
        self.pending = {"id": request_id, "kind": kind, "name": name,
                        "deadline": time.monotonic() + 8, "count": None}
        self.received_parameters = {}
        self.worker.send(command.encode("ascii"))
        self.status.set("正在读取参数…" if kind == "READ" else f"正在保存 {name} 到内存和 SD…")
        self._update_controls()

    def read_parameters(self):
        self._start_request("READ")

    def write_parameter(self):
        selection = self.tree.selection()
        if not selection or not self.connected or self.pending:
            return
        parameter = self.parameters[selection[0]]
        try:
            value = validate_edit(parameter, self.new_value.get().strip())
        except ValueError as exc:
            self.status.set(str(exc))
            return
        self._start_request("SET", parameter.name, value)

    def _select_parameter(self, _=None):
        selection = self.tree.selection()
        if self.editing_name:
            return
        if selection and selection[0] in self.parameters:
            parameter = self.parameters[selection[0]]
            self.selected_name.set(parameter.name)
            self.new_value.set(format_value(parameter.value, parameter.dtype))
        else:
            self.selected_name.set("未选择参数")
            self.new_value.set("")
        self._update_controls()

    def _render_table(self):
        self._cancel_edit()
        selection = self.tree.selection()
        self.tree.delete(*self.tree.get_children())
        text = self.filter_text.get().lower().strip()
        for name, p in self.parameters.items():
            if text and text not in f"{p.group} {name} {p.description}".lower():
                continue
            self.tree.insert("", "end", iid=name, values=(p.group, name,
                             p.dtype, format_value(p.value, p.dtype),
                             f"{format_value(p.minimum, p.dtype)} ～ {format_value(p.maximum, p.dtype)}", p.description))
        self._apply_sort()
        if selection and self.tree.exists(selection[0]):
            self.tree.selection_set(selection[0])

    def _sort_heading(self, column):
        self._cancel_edit()
        self.sort_reverse = not self.sort_reverse if self.sort_column == column else False
        self.sort_column = column
        labels = dict(zip(self.tree["columns"], ("分组", "参数名", "类型", "当前值", "允许范围", "说明")))
        for key, label in labels.items():
            self.tree.heading(key, text=label + (" ▼" if self.sort_reverse else " ▲") if key == column else label)
        self._apply_sort()

    def _apply_sort(self):
        if not self.sort_column:
            return
        def key(name):
            p = self.parameters[name]
            if self.sort_column == "value":
                return p.value
            if self.sort_column == "bounds":
                return (p.minimum, p.maximum)
            text = getattr(p, self.sort_column).casefold()
            return tuple((0, int(part)) if part.isdigit() else (1, part)
                         for part in re.split(r"([0-9]+)", text))
        for index, name in enumerate(sorted(self.tree.get_children(), key=key, reverse=self.sort_reverse)):
            self.tree.move(name, "", index)

    def _finish_request(self, message):
        self.pending = None
        self.status.set(message)
        self._update_controls()

    def _handle_reply(self, reply):
        request_id, kind, fields = reply
        if not self.pending or request_id != self.pending["id"]:
            return
        try:
            if kind == "ERROR" and len(fields) == 2:
                messages = {
                    "NO_SD": "SD 卡不可用，内存未修改",
                    "SD_WRITE": "SD 保存或读回校验失败，内存未修改",
                    "RANGE": "数值超出范围或不是有效数字",
                }
                self._finish_request("失败：" + messages.get(fields[0], " / ".join(fields)))
            elif self.pending["kind"] == "READ":
                if kind == "BEGIN" and len(fields) in (4, 7):
                    count = int(fields[1])
                    if not 0 < count <= MAX_PARAMETERS:
                        raise ValueError("参数数量无效")
                    self.pending.update(count=count, profile=fields[0], storage=fields[2], source=fields[3])
                    if len(fields) == 7:
                        matched, defaults_used, skipped = map(int, fields[4:7])
                        if min(matched, defaults_used, skipped) < 0 or matched + defaults_used != count:
                            raise ValueError("参数迁移统计无效")
                        self.pending["migration_stats"] = (matched, defaults_used, skipped)
                    self.received_parameters.clear()
                elif kind == "VALUE" and self.pending["count"] is not None:
                    parameter = parameter_from_fields(fields)
                    if parameter.name in self.received_parameters:
                        raise ValueError("参数响应包含重复名称")
                    self.received_parameters[parameter.name] = parameter
                elif kind == "END" and len(fields) == 1:
                    if (self.pending["count"] is None or
                            int(fields[0]) != self.pending["count"] or
                            len(self.received_parameters) != self.pending["count"]):
                        raise ValueError("参数响应不完整，请重新读取")
                    self.parameters = dict(self.received_parameters)
                    self._render_table()
                    pending = self.pending
                    storage = "SD 就绪" if pending["storage"] == "SD_READY" else "SD 不可用"
                    source = {"LOADED": "从 SD 加载", "SAVED": "已保存到 SD", "DEFAULTS": "源码默认值",
                              "MIGRATED": "已迁移到新参数表", "MIGRATION_PENDING": "已加载旧值，SD 迁移待完成"}.get(
                        pending["source"], pending["source"])
                    stats = pending.get("migration_stats")
                    details = (f" · 开机匹配 {stats[0]} / 默认 {stats[1]} / 跳过旧项 {stats[2]}"
                               if stats and (stats[1] or stats[2]) else "")
                    self._finish_request(f"已读取 {len(self.parameters)} 项 · {pending['profile']} · {storage} · {source}{details}")
            elif kind == "OK" and len(fields) in (3, 4):
                if len(fields) == 4:
                    name, dtype, text, saved = fields
                    if dtype != self.parameters[name].dtype:
                        raise ValueError("写入确认的类型不匹配")
                else:
                    name, text, saved = fields
                if name != self.pending["name"] or saved != "SAVED":
                    raise ValueError("写入确认内容不匹配")
                value = validate_edit(self.parameters[name], text)
                self.parameters[name] = replace(self.parameters[name], value=value)
                self._render_table()
                self.new_value.set(f"{format_value(value, self.parameters[name].dtype)}")
                self._finish_request(f"{name} = {format_value(value, self.parameters[name].dtype)}，已写入内存并保存 SD")
        except (ValueError, KeyError) as exc:
            self._finish_request(f"响应错误：{exc}")

    def send_command(self):
        if not self.connected or not self.command.get():
            return
        self.worker.send((self.command.get() + "\n").encode("utf-8"))

    def send_help(self):
        if self.connected:
            self.worker.send(b"PARAM HELP\n")

    def _append_terminal(self, text):
        if not text:
            return
        self.terminal.configure(state="normal")
        self.terminal.insert("end", text)
        # The raw recording preserves all bytes; the UI keeps recent history.
        lines = int(self.terminal.index("end-1c").split(".")[0])
        if lines > 5000:
            self.terminal.delete("1.0", f"{lines - 5000}.0")
        self.terminal_chars += len(text)
        if self.terminal_chars > 500_000:
            self.terminal.delete("1.0", "end-250000c")
            self.terminal_chars = int(self.terminal.count("1.0", "end-1c", "chars")[0])
        if self.follow.get():
            self.terminal.see("end")
        self.terminal.configure(state="disabled")

    def _clear_terminal(self):
        self.history.clear()
        self.history_size = 0
        self.display_pending.clear()
        self.terminal_chars = 0
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")
        self.terminal.configure(state="normal")
        self.terminal.delete("1.0", "end")
        self.terminal.configure(state="disabled")

    def _display_bytes(self, kind, payload):
        if self.hex_display.get():
            label = "接收" if kind == "rx" else "发送"
            return "".join(f"[{label}] " + payload[i:i+16].hex(" ").upper() + "\n"
                           for i in range(0, len(payload), 16))
        if kind == "rx":
            return self.decoder.decode(payload)
        return "\n[发送] " + payload.decode("utf-8", errors="replace")

    def _remember_bytes(self, kind, payload):
        self.history.append((kind, payload))
        self.history_size += len(payload)
        while self.history_size > 128_000 and self.history:
            self.history_size -= len(self.history.popleft()[1])
        return self._display_bytes(kind, payload)

    def _rebuild_terminal(self):
        self.display_pending.clear()
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")
        self.terminal.configure(state="normal")
        self.terminal.delete("1.0", "end")
        self.terminal.configure(state="disabled")
        self.terminal_chars = 0
        self._append_terminal("".join(self._display_bytes(kind, payload) for kind, payload in self.history))

    def export_terminal(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("文本", "*.txt")])
        if path:
            try:
                Path(path).write_text(self.terminal.get("1.0", "end-1c"), encoding="utf-8")
            except OSError as exc:
                messagebox.showerror("导出失败", str(exc))

    def toggle_recording(self):
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            self.record_button.configure(text="记录全部接收")
            self.status.set("接收记录已保存")
            return
        path = filedialog.asksaveasfilename(defaultextension=".log", filetypes=[("原始串口记录", "*.log")])
        if path:
            try:
                self.log_file = open(path, "ab", buffering=0)
                self.record_button.configure(text="停止记录")
                self.status.set(f"正在记录接收到的全部原始字节：{path}")
            except OSError as exc:
                messagebox.showerror("记录失败", str(exc))

    def _poll(self):
        terminal_chunks = []
        poll_start = time.monotonic()
        for _ in range(200):
            if time.monotonic() - poll_start > 0.006:
                break
            try:
                connection_id, kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if connection_id != self.connection_id:
                continue
            if kind == "connected":
                self.connected = True
                self.status.set("已连接；点击“读取参数”获取飞控当前值")
                self._update_controls()
            elif kind == "rx":
                if self.log_file:
                    try:
                        self.log_file.write(payload)
                    except OSError as exc:
                        self.log_file.close()
                        self.log_file = None
                        self.record_button.configure(text="记录全部接收")
                        self.status.set(f"接收记录失败：{exc}")
                terminal_chunks.append(self._remember_bytes("rx", payload))
                for reply in self.framer.feed(payload):
                    self._handle_reply(reply)
            elif kind == "tx":
                terminal_chunks.append(self._remember_bytes("tx", payload))
            elif kind == "error":
                unconfirmed_write = self.pending and self.pending["kind"] == "SET"
                self.disconnect()
                self.status.set(f"串口连接失败或中断：{payload}")
                if unconfirmed_write:
                    self.status.set("串口中断，写入结果未确认；请重新连接并读取参数")
                terminal_chunks.append(f"\n[连接错误] {payload}\n")
            elif kind == "closed" and self.worker:
                self.disconnect()
        self.display_pending.extend(terminal_chunks)
        if time.monotonic() - self.last_display >= 0.1:
            self._append_terminal("".join(self.display_pending))
            self.display_pending.clear()
            self.last_display = time.monotonic()
        if self.pending and time.monotonic() > self.pending["deadline"]:
            operation = self.pending["kind"]
            self._finish_request("写入结果未确认，请重新读取参数；不要重复发送写入命令" if operation == "SET"
                                 else "读取超时，请检查固件版本和串口后重新读取")
        self.poll_timer = self.root.after(30, self._poll)

    def close(self):
        self.root.after_cancel(self.poll_timer)
        if self.worker:
            self.worker.stop()
            self.worker.join(timeout=0.75)
        if self.log_file:
            self.log_file.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    self_test = "--self-test" in sys.argv[1:]
    if self_test:
        root.withdraw()
    console = ParameterConsole(root)
    if self_test:
        root.after(200, console.close)
    root.mainloop()


if __name__ == "__main__":
    main()
