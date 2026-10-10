"""CoFly Ground Station: one USB connection for parameters and live data."""
import json
import math
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk, font

from parameter_view import ParameterConsole
from station_protocol import MODES, decode_fields
from wing_geometry import wing_segments, configuration_ready

SETTINGS_PATH = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "CoFly" / "ground_station.json"


class GroundStation(ParameterConsole):
    def __init__(self, root, settings_path=None):
        self.settings_path = Path(settings_path) if settings_path else SETTINGS_PATH
        try:
            settings = json.loads(self.settings_path.read_text(encoding="utf-8"))
            if not isinstance(settings, dict):
                settings = {}
        except (OSError, ValueError):
            settings = {}
        self.theme = tk.StringVar(root, value=settings.get("theme") if settings.get("theme") in ("室内", "室外") else "室内")
        size = settings.get("font_size", 11)
        self.font_size = tk.IntVar(root, value=size if type(size) is int and 9 <= size <= 18 else 11)
        self.pages = {}
        self.nav = {}
        self.caps = {}
        self.gcs_pending = None
        self.last_data = 0
        self.data = {}
        self.configuration = {}
        self.last_configuration = 0
        self.config_supported = True
        self.auto_probe = True
        self.page = "飞行"
        self.metric_vars = {}
        self.action_buttons = []
        super().__init__(root)
        root.title("CoFly 地面站 · USB Serial")
        root.geometry("1280x850")
        root.minsize(1000, 720)
        self.port.set(str(settings.get("port", self.port.get())))
        self.baud.set(str(settings.get("baud", "921600")))
        self.apply_appearance(save=False)
        self.show_page("飞行")
        self.station_timer = root.after(200, self._station_tick)

    def _build_widgets(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        shell = ttk.Frame(self.root)
        shell.pack(fill="both", expand=True)
        top = ttk.Frame(shell, padding=(20, 12))
        top.pack(fill="x")
        ttk.Label(top, text="CoFly", style="Brand.TLabel").pack(side="left")
        ttk.Label(top, text="  地面站 / USB SERIAL", style="Muted.TLabel").pack(side="left")
        self.link_label = tk.StringVar(value="● 未连接")
        ttk.Label(top, textvariable=self.link_label, style="Accent.TLabel").pack(side="right")
        ttk.Label(shell, textvariable=self.status, padding=(20, 10), wraplength=1150).pack(side="bottom", fill="x")
        body = ttk.Frame(shell)
        body.pack(fill="both", expand=True)
        sidebar = ttk.Frame(body, padding=12, width=170)
        sidebar.pack(side="left", fill="y")
        content = ttk.Frame(body, padding=(12, 8))
        content.pack(side="left", fill="both", expand=True)
        for section, names in (("飞行", ("飞行",)), ("飞机设置", ("参数", "校准", "遥控器", "串口助手")),
                               ("应用设置", ("通讯连接", "应用设置"))):
            ttk.Label(sidebar, text=section, style="Muted.TLabel").pack(anchor="w", pady=(18, 8))
            for name in names:
                self.nav[name] = ttk.Button(sidebar, text=name, width=16, command=lambda n=name: self.show_page(n))
                self.nav[name].pack(fill="x", pady=3, ipady=6)
                page = ttk.Frame(content)
                page.grid(row=0, column=0, sticky="nsew")
                self.pages[name] = page
        content.columnconfigure(0, weight=1)
        content.rowconfigure(0, weight=1)
        self.parameter_parent = self.pages["参数"]
        self._heading(self.pages["串口助手"], "串口助手", "共用通讯连接页的 USB 串口 · 接收日志、发送命令与导出记录")
        self.terminal_parent = self.pages["串口助手"]
        self.connection_parent = ttk.Frame(self.pages["通讯连接"], padding=12)
        self._heading(self.pages["通讯连接"], "通讯连接", "使用飞控默认 USB Serial，默认波特率 921600。请关闭占用该端口的串口监视器。")
        self.connection_parent.pack(fill="x", pady=12)
        self.connection_parent.columnconfigure(0, weight=1)
        super()._build_widgets()
        parameter_actions = ttk.Frame(self.parameter_parent)
        parameter_actions.pack(before=self.parameter_parent.winfo_children()[0], fill="x", padx=12, pady=(8, 0))
        self.parameter_read = self.read_button = ttk.Button(parameter_actions, text="读取参数", command=self.read_parameters)
        self.parameter_read.pack(side="left")
        self.parameter_load_file = ttk.Button(parameter_actions, text="从文件读取", command=self.load_parameters_from_file)
        self.parameter_load_file.pack(side="left", padx=8)
        self.parameter_save_file = ttk.Button(parameter_actions, text="保存到文件", command=self.save_parameters_to_file)
        self.parameter_save_file.pack(side="left")
        self._build_flight()
        self._build_calibration()
        self._build_radio()
        self._build_settings()
        self.device_label = tk.StringVar(value="固件能力尚未识别")
        ttk.Label(self.pages["通讯连接"], textvariable=self.device_label, wraplength=740).pack(anchor="w", padx=12, pady=12)
        self.probe_button = ttk.Button(self.pages["通讯连接"], text="识别固件 / 重试", command=lambda: self._request("CAPS"))
        self.reboot_button = ttk.Button(self.pages["通讯连接"], text="重启飞控", command=lambda: self._request("REBOOT"), state="disabled")
        self.reboot_button.pack(anchor="w", padx=12, pady=6)
        ttk.Label(self.pages["通讯连接"], text="重启前需锁定飞机；设备重启后请重新连接 USB。").pack(anchor="w", padx=12)
        self.probe_button.pack(anchor="w", padx=12)
        ttk.Label(self.pages["通讯连接"], text="连接后自动识别固件。旧固件仍可使用参数页；数据与传感器页需要新版固件。", wraplength=740).pack(anchor="w", padx=12, pady=18)

    def _heading(self, page, title, subtitle):
        ttk.Label(page, text=title, style="Title.TLabel").pack(anchor="w", pady=(12, 6))
        ttk.Label(page, text=subtitle, style="Muted.TLabel", wraplength=740).pack(anchor="w", pady=(0, 18))

    def _build_flight(self):
        page = self.pages["飞行"]
        self._heading(page, "飞行数据", "地面 USB 实时监测 · 5 Hz · 所有数值来自飞控，未提供的数据以 — 显示")
        self.freshness = tk.StringVar(value="等待连接")
        ttk.Label(page, textvariable=self.freshness, style="Accent.TLabel").pack(anchor="w", pady=(0, 12))
        dashboard = ttk.Frame(page)
        dashboard.pack(fill="both", expand=True)
        left = ttk.Labelframe(dashboard, text="姿态", padding=12)
        left.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=(0, 12), pady=6)
        self.horizon = tk.Canvas(left, width=300, height=280, highlightthickness=0)
        self.horizon.pack(fill="both", expand=True)
        self.attitude_text = tk.StringVar(value="ROLL —   PITCH —   YAW —")
        ttk.Label(left, textvariable=self.attitude_text, anchor="center").pack(before=self.horizon, side="bottom", fill="x", pady=6)
        groups = (("角速度 · °/s", (("gx", "X"), ("gy", "Y"), ("gz", "Z"))),
                  ("加速度 · g", (("ax", "X"), ("ay", "Y"), ("az", "Z"))),
                  ("速度 / 位置", (("airspeed", "空速 m/s"), ("groundspeed", "地速 m/s"), ("altitude", "高度 m"))),
                  ("飞机状态", (("mode", "模式"), ("locked", "锁定"), ("angle", "构型角 °"))))
        for index, (title, metrics) in enumerate(groups):
            card = ttk.Labelframe(dashboard, text=title, padding=(10, 5))
            card.grid(row=index // 2, column=1 + index % 2, sticky="nsew", padx=6, pady=6)
            for key, label in metrics:
                row = ttk.Frame(card)
                row.pack(fill="x", pady=3)
                ttk.Label(row, text=label, style="Muted.TLabel").pack(side="left")
                var = self.metric_vars[key] = tk.StringVar(value="—")
                ttk.Label(row, textvariable=var, style="Value.TLabel").pack(side="right")
        for col in range(3):
            dashboard.columnconfigure(col, weight=1)
        for row in range(2):
            dashboard.rowconfigure(row, weight=1)
        self.horizon.bind("<Configure>", lambda _: self._draw_horizon())
        flight_note = ttk.Label(page, text="地速、高度及硬件健康状态尚无有效数据接口。传感器列表表示固件采样配置。", style="Muted.TLabel")
        flight_note.pack(before=dashboard, side="bottom", anchor="w", pady=8)
        formation = ttk.Labelframe(page, text="实时构型 · 从后向前看 · 翼尖连接", padding=8)
        formation.pack(before=flight_note, side="bottom", fill="x", pady=(0, 6))
        self.configuration_label = tk.StringVar(value="等待构型数据（相邻飞机 IMU 滚转差）")
        ttk.Label(formation, textvariable=self.configuration_label, style="Muted.TLabel").pack(anchor="w")
        self.wings = tk.Canvas(formation, height=155, highlightthickness=0)
        self.wings.pack(fill="x", expand=True)
        self.wings.bind("<Configure>", lambda _: self._draw_configuration())

    def _build_calibration(self):
        page = self.pages["校准"]
        self._heading(page, "传感器校准", "识别当前固件的实际采样路径；USB 校准与传感器设置接口预留至下一版本。")
        self.sensor_tree = ttk.Treeview(page, columns=("sensor", "state"), show="headings", height=7, selectmode="browse")
        self.sensor_tree.heading("sensor", text="传感器 / 来源")
        self.sensor_tree.heading("state", text="配置与可用性")
        self.sensor_tree.column("sensor", width=210)
        self.sensor_tree.column("state", width=520)
        self.sensor_tree.pack(fill="x", pady=12)
        controls = ttk.Frame(page)
        controls.pack(fill="x", pady=12)
        self.sensor_setting = tk.StringVar(value="DEFAULT")
        ttk.Label(controls, text="设置预留项").pack(side="left")
        ttk.Combobox(controls, textvariable=self.sensor_setting, values=("DEFAULT",), state="readonly", width=15).pack(side="left", padx=10)
        for label, verb in (("校准所选传感器（预留）", "CALIBRATE"), ("应用设置（预留）", "SENSOR_SET")):
            button = ttk.Button(controls, text=label, command=lambda v=verb: self._sensor_action(v))
            button.pack(side="left", padx=4)
            self.action_buttons.append(button)
        self.sensor_note = tk.StringVar(value="连接并识别固件后显示传感器。")
        ttk.Label(page, textvariable=self.sensor_note, wraplength=740).pack(anchor="w", pady=12)

    def _build_radio(self):
        page = self.pages["遥控器"]
        self._heading(page, "遥控器", "实时显示现有六路输入；校准和飞行模式映射通过 V2 预留接口提交。")
        self.radio_label = tk.StringVar(value="接收机：未识别")
        ttk.Label(page, textvariable=self.radio_label).pack(anchor="w", pady=8)
        self.rc_bars = []
        for index, name in enumerate(("滚转", "俯仰", "油门", "偏航", "油门切断", "辅助"), 1):
            row = ttk.Frame(page)
            row.pack(fill="x", pady=8)
            ttk.Label(row, text=f"CH{index}  {name}", width=18).pack(side="left")
            bar = ttk.Progressbar(row, maximum=1000)
            bar.pack(side="left", fill="x", expand=True, padx=16)
            var = self.metric_vars[f"rc{index}"] = tk.StringVar(value="— μs")
            ttk.Label(row, textvariable=var, width=12).pack(side="right")
            self.rc_bars.append(bar)
        button = ttk.Button(page, text="遥控器校准（预留）", command=lambda: self._request("RADIO_CALIBRATE"))
        button.pack(anchor="w", pady=16)
        self.action_buttons.append(button)
        mapping = ttk.Labelframe(page, text="模式开关映射（下一版本实现）", padding=16)
        mapping.pack(fill="x", pady=8)
        self.mode_vars = []
        for label, mode in zip(("低位", "中位", "高位"), MODES.values()):
            ttk.Label(mapping, text=label).pack(side="left", padx=8)
            var = tk.StringVar(value=mode)
            ttk.Combobox(mapping, textvariable=var, values=tuple(MODES.values()), state="readonly", width=15).pack(side="left")
            self.mode_vars.append(var)
        button = ttk.Button(page, text="提交模式映射（预留）", command=self._mode_action)
        button.pack(anchor="w", pady=10)
        self.action_buttons.append(button)

    def _build_settings(self):
        page = self.pages["应用设置"]
        self._heading(page, "应用设置", "室内深色与室外高对比浅色；设置自动保存到本机。")
        row = ttk.Frame(page)
        row.pack(fill="x", pady=12)
        ttk.Label(row, text="配色", width=16).pack(side="left")
        for value in ("室内", "室外"):
            ttk.Radiobutton(row, text=value, variable=self.theme, value=value, command=self.apply_appearance).pack(side="left", padx=12)
        row = ttk.Frame(page)
        row.pack(fill="x", pady=12)
        ttk.Label(row, text="字体大小", width=16).pack(side="left")
        box = ttk.Combobox(row, textvariable=self.font_size, values=tuple(range(9, 19)), state="readonly", width=8)
        box.pack(side="left", padx=12)
        box.bind("<<ComboboxSelected>>", lambda _: self.apply_appearance())

    def show_page(self, name):
        self._cancel_edit()
        self.page = name
        self.pages[name].tkraise()
        for key, button in self.nav.items():
            button.configure(style="Selected.TButton" if key == name else "TButton")

    def apply_appearance(self, save=True):
        dark = self.theme.get() == "室内"
        bg, panel, fg, muted, accent = ("#151c26", "#202b39", "#edf3fa", "#a9b8ca", "#75c8f4") if dark else ("#edf1f5", "#ffffff", "#142539", "#42566c", "#075e91")
        self.colors = (bg, panel, fg, muted, accent)
        size = self.font_size.get()
        self.root.minsize(max(1000, size * 90), max(850, size * 60))
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            font.nametofont(name).configure(family="Microsoft YaHei UI", size=size)
        style = ttk.Style(self.root)
        style.configure(".", background=bg, foreground=fg, font=("Microsoft YaHei UI", size))
        style.configure("TFrame", background=bg)
        style.configure("TLabel", background=bg, foreground=fg)
        style.configure("TLabelframe", background=bg, bordercolor=muted)
        style.configure("TLabelframe.Label", background=bg, foreground=accent)
        for kind in ("TButton", "TCombobox", "TEntry", "Treeview", "Treeview.Heading"):
            style.configure(kind, background=panel, fieldbackground=panel, foreground=fg, insertcolor=fg)
            style.map(kind, background=[("selected", "#245d85"), ("active", "#245d85")], foreground=[("disabled", muted), ("selected", "white"), ("active", "white")])
        style.map("TCombobox", fieldbackground=[("readonly", panel), ("disabled", bg)],
                  foreground=[("disabled", muted), ("readonly", fg)],
                  selectbackground=[("readonly", panel)], selectforeground=[("readonly", fg)])
        style.configure("Horizontal.TProgressbar", background=accent, troughcolor=panel, bordercolor=muted)
        style.configure("Treeview", rowheight=size * 2 + 8)
        style.configure("Selected.TButton", background="#245d85", foreground="white")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", size + 10, "bold"))
        style.configure("Brand.TLabel", foreground=accent, font=("Microsoft YaHei UI", size + 9, "bold"))
        style.configure("Muted.TLabel", foreground=muted)
        style.configure("Accent.TLabel", foreground=accent)
        style.configure("Value.TLabel", font=("Microsoft YaHei UI", size + 3, "bold"))
        self.terminal.configure(background=panel, foreground=fg, insertbackground=fg, font=("Consolas", size))
        self.root.configure(background=bg)
        self._draw_horizon()
        self._draw_configuration()
        if save:
            self._save_settings()

    def _save_settings(self):
        try:
            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.settings_path.with_suffix(".tmp")
            temporary.write_text(json.dumps({"theme": self.theme.get(), "font_size": self.font_size.get(), "port": self.port.get(), "baud": self.baud.get()}, ensure_ascii=False), encoding="utf-8")
            temporary.replace(self.settings_path)
        except OSError as exc:
            self.status.set(f"设置保存失败：{exc}")

    def _update_controls(self):
        super()._update_controls()
        if hasattr(self, "parameter_read"):
            idle = self.connected and self.pending is None and self.gcs_pending is None and self.file_import is None
            self.read_button.configure(state="normal" if idle else "disabled")
            self.parameter_read.configure(state="normal" if idle else "disabled")
            files_ready = idle and bool(self.parameters) and bool(self.parameter_profile)
            self.parameter_load_file.configure(state="normal" if files_ready else "disabled")
            self.parameter_save_file.configure(state="normal" if files_ready else "disabled")
            self.probe_button.configure(state="normal" if idle else "disabled")
            self.reboot_button.configure(state="normal" if idle and self.caps.get("reboot") == "SUPPORTED" else "disabled")
            for button in self.action_buttons:
                button.configure(state="normal" if idle and self.caps else "disabled")
        self.link_label.set("● USB 已连接" if self.connected else "● 未连接")

    def _start_request(self, *args):
        if self.gcs_pending:
            self.status.set("正在接收 USB 数据，请稍后再试")
            return
        super()._start_request(*args)

    def _request(self, verb, *args):
        if not self.connected or self.pending or self.gcs_pending or self.file_import is not None:
            return
        self.request_id = (self.request_id + 1) & 0xffffffff
        self.gcs_pending = {"id": self.request_id, "verb": verb, "deadline": time.monotonic() + 3}
        self.worker.send((f"GCS {verb} {self.request_id}" + (" " + " ".join(args) if args else "") + "\n").encode("ascii"))
        self._update_controls()

    def _sensor_action(self, verb):
        selected = self.sensor_tree.selection()
        active = {"IMU": self.caps.get("imu") == "BMI088_SPI", "EXTIMU": self.caps.get("external") == "ENABLED",
                  "AIRSPEED": self.caps.get("airspeed") == "MS4525", "ROTATE": self.caps.get("rotate") == "MT6701"}
        if not selected or not active.get(selected[0], False):
            self.status.set("请选择当前启用且正在采样的传感器")
            return
        args = (selected[0],) if verb == "CALIBRATE" else (selected[0], self.sensor_setting.get())
        self._request(verb, *args)

    def _mode_action(self):
        codes = {label: code for code, label in MODES.items()}
        self._request("FLIGHT_MODES", *(codes[var.get()] for var in self.mode_vars))

    def _handle_reply(self, reply):
        request_id, kind, fields = reply
        pending = self.gcs_pending
        if not pending or request_id != pending["id"]:
            return super()._handle_reply(reply)
        self.gcs_pending = None
        try:
            if kind == "ERROR":
                code = fields[0] if fields else "UNKNOWN"
                self.status.set("该接口暂未实现，待下一版本开放；飞控状态未修改。" if code == "NOT_IMPLEMENTED" else
                                "请先锁定飞机，再重启飞控" if code == "LOCK_REQUIRED" else f"固件拒绝地面站请求：{code}")
                if pending["verb"] == "CAPS":
                    self.device_label.set("固件不支持地面站协议；参数页仍可使用。请更新固件后重试识别。")
                elif pending["verb"] == "CONFIG":
                    self.config_supported = False
                    self.configuration_label.set("固件不支持构型接口，请更新固件")
            else:
                if kind != pending["verb"]:
                    raise ValueError("回复类型与请求不匹配")
                values = decode_fields(kind, fields)
                if kind == "REBOOT":
                    self.disconnect()
                    self.status.set("飞控已接受重启指令；设备重启后请重新连接 USB")
                if kind == "CAPS":
                    self.caps = values
                    self._render_sensors()
                    self.device_label.set(f"CoFly USB 协议 v1 · {values['imu']} · {values['radio']} · 校准与设置接口预留")
                    self.radio_label.set(f"接收机：{values['radio']} · 显示固件处理后的通道，非接收机原始帧")
                    self.status.set("固件已识别，USB 数据面板已启用")
                elif kind == "DATA":
                    self.data = values
                    self.last_data = time.monotonic()
                    self._render_data()
                elif kind == "CONFIG":
                    self.configuration = values
                    self.last_configuration = time.monotonic()
                    if "aircraft_id" in values:
                        identity = chr(64 + values["aircraft_id"]) if values["config_valid"] else "未配置"
                        self.device_label.set(f"当前生效身份：{identity} · {values['count']} 机 · USB 协议 v1")
                    self._draw_configuration()
        except (ValueError, IndexError) as exc:
            self.status.set(f"地面站响应错误：{exc}")
        self._update_controls()
        if kind == "DATA" and self.page == "飞行" and self.config_supported:
            self._request("CONFIG")

    def _render_sensors(self):
        self.sensor_tree.delete(*self.sensor_tree.get_children())
        for key, name, state in (("IMU", "内置 IMU / 陀螺仪 / 加速度计", self.caps["imu"]),
                                  ("EXTIMU", "外置 IMU", self.caps["external"]),
                                  ("AIRSPEED", "空速", self.caps["airspeed"]),
                                  ("ROTATE", "构型转角", self.caps["rotate"]),
                                  ("BAROMETER", "气压计", self.caps["barometer"])):
            self.sensor_tree.insert("", "end", iid=key, values=(name, state))
        self.sensor_note.set("识别的是编译配置与采样路径，未进行硬件探测。校准、设置按钮只请求预留接口，不调用现有阻塞式校准。")

    def _render_data(self):
        for key, var in self.metric_vars.items():
            value = self.data.get(key)
            if key == "mode":
                text = MODES.get(value, "—")
            elif key == "locked":
                text = "已锁定" if value == 1 else "未锁定" if value == 0 else "—"
            elif key == "angle" and self.data.get("angle_valid") != 1:
                text = "无效"
            elif key.startswith("rc"):
                text = f"{value:.0f} μs" if value is not None else "— μs"
            elif key in ("gx", "gy", "gz", "ax", "ay", "az") and self.caps.get("imu") != "BMI088_SPI":
                text = "—"
            else:
                text = f"{value:.2f}" if value is not None else "—"
            var.set(text)
        for index, bar in enumerate(self.rc_bars, 1):
            bar["value"] = max(0, min(1000, self.data.get(f"rc{index}", 1000) - 1000))
        self.attitude_text.set("   ".join(f"{key.upper()} {self.data[key]:.1f}°" for key in ("roll", "pitch", "yaw")) if self.data else "ROLL —   PITCH —   YAW —")
        self._draw_horizon()

    def _draw_horizon(self):
        if not hasattr(self, "horizon"):
            return
        canvas = self.horizon
        canvas.delete("all")
        w, h = max(canvas.winfo_width(), 100), max(canvas.winfo_height(), 100)
        canvas.configure(background="#263442")
        if not self.data:
            canvas.create_text(w / 2, h / 2, text="等待姿态数据", fill="#d1deea", font=("Microsoft YaHei UI", 15))
            return
        cx, cy = w / 2, h / 2
        angle = math.radians(self.data["roll"])
        pitch = max(-60, min(60, self.data["pitch"])) * h / 120
        extent = max(w, h) * 3
        def point(x, y):
            return (cx + x * math.cos(angle) - (y + pitch) * math.sin(angle),
                    cy + x * math.sin(angle) + (y + pitch) * math.cos(angle))
        for bounds, color in (((-extent, -extent, extent, 0), "#276b96"), ((-extent, 0, extent, extent), "#866547")):
            x0, y0, x1, y1 = bounds
            canvas.create_polygon(*sum((point(x, y) for x, y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))), ()), fill=color, outline="")
        canvas.create_line(*point(-extent, 0), *point(extent, 0), fill="white", width=2)
        for tick in (-30, -20, -10, 10, 20, 30):
            y = -tick * h / 120
            canvas.create_line(*point(-22, y), *point(22, y), fill="white")
            canvas.create_text(*point(40, y), text=str(tick), fill="white")
        canvas.create_line(cx-70, cy, cx-20, cy, cx-12, cy+9, cx, cy, cx+12, cy+9, cx+20, cy, cx+70, cy, fill="#ffda64", width=3)

    def _draw_configuration(self):
        if not hasattr(self, "wings"):
            return
        canvas = self.wings
        canvas.delete("all")
        bg, panel, fg, muted, accent = self.colors
        canvas.configure(background=panel)
        w, h = max(200, canvas.winfo_width()), max(120, canvas.winfo_height())
        if not configuration_ready(self.configuration):
            if not self.config_supported:
                message = "请更新支持构型接口的固件"
            elif self.configuration and self.configuration.get("config_valid") == 0:
                message = "请在参数页设置飞机编号和数量，然后重启"
            elif self.configuration and not self.configuration["master"]:
                message = "完整构型需连接 A 主机"
            else:
                message = "等待有效的相邻飞机姿态"
            canvas.create_text(w / 2, h / 2, text=message, fill=muted, font=("Microsoft YaHei UI", self.font_size.get()))
            self.configuration_label.set(message)
            return
        segments = wing_segments(self.configuration)
        points = [point for _, left, right, _ in segments for point in (left, right)]
        xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
        ymin, ymax = min(p[1] for p in points), max(p[1] for p in points)
        # Fit the whole chain, including near-vertical or folded wings.
        scale = min((w-90) / max(.4, xmax-xmin), (h-70) / max(.4, ymax-ymin))
        def screen(point):
            return w/2 + (point[0]-(xmin+xmax)/2)*scale, h/2 + (point[1]-(ymin+ymax)/2)*scale
        canvas.create_line(25, h/2, w-25, h/2, fill=muted, dash=(3, 6))
        for name, left, right, angle in segments:
            x0, y0 = screen(left)
            x1, y1 = screen(right)
            color = accent if name != "A" else ("#ffda64" if self.theme.get() == "室内" else "#855300")
            canvas.create_line(x0, y0, x1, y1, fill=color, width=6, capstyle="round", tags=("wing", name))
            canvas.create_oval(x0-3, y0-3, x0+3, y0+3, fill=fg, outline="")
            canvas.create_oval(x1-3, y1-3, x1+3, y1+3, fill=fg, outline="")
            canvas.create_text((x0+x1)/2, (y0+y1)/2+22, text=f"{name}  {angle:.1f}°", fill=fg,
                               font=("Microsoft YaHei UI", self.font_size.get()))
        self.configuration_label.set(f"{self.configuration['count']} 机 · " + "—".join(s[0] for s in segments) +
                                     " · 相对角来源：相邻 IMU 滚转差（不使用 MT6701）")

    def _station_tick(self):
        now = time.monotonic()
        if self.gcs_pending and now > self.gcs_pending["deadline"]:
            verb = self.gcs_pending["verb"]
            self.gcs_pending = None
            self.status.set("地面站请求超时；请检查固件或重试识别" if verb == "CAPS" else "USB 请求超时，结果未确认")
            self._update_controls()
        if self.connected and not self.pending and not self.gcs_pending and self.file_import is None:
            if self.auto_probe:
                self.auto_probe = False
                self._request("CAPS")
            elif self.caps and self.page in ("飞行", "遥控器"):
                self._request("DATA")
        age = now - self.last_data if self.last_data else float("inf")
        fresh = self.connected and age <= 2
        self.freshness.set(f"● 实时 · 更新于 {age:.1f} 秒前" if fresh else "● 数据已过期 / 等待 USB 数据")
        if not fresh and self.data:
            self.data.clear()
            self._render_data()
        if self.configuration and now - self.last_configuration > 2:
            self.configuration.clear()
            self._draw_configuration()
        self.station_timer = self.root.after(200, self._station_tick)

    def disconnect(self):
        self.gcs_pending = None
        self.caps.clear()
        self.data.clear()
        self.configuration.clear()
        self.last_configuration = 0
        self.config_supported = True
        self.last_data = 0
        self.auto_probe = True
        super().disconnect()
        self._render_data()
        self._draw_configuration()
        self.sensor_tree.delete(*self.sensor_tree.get_children())
        self.device_label.set("固件能力尚未识别")
        self.radio_label.set("接收机：未识别")
        self.sensor_note.set("连接并识别固件后显示传感器。")

    def close(self):
        self.root.after_cancel(self.station_timer)
        self._save_settings()
        super().close()


def main():
    root = tk.Tk()
    station = GroundStation(root)
    if "--self-test" in sys.argv:
        root.withdraw()
        root.after(300, station.close)
    root.mainloop()


if __name__ == "__main__":
    main()
