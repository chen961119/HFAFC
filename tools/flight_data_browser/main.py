"""CoFly Autopilot flight-data browser and command-line reader."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
from pathlib import Path
import queue
import sys

import numpy as np

from data import LogError, compute_spectrum, describe_variable, display_series, load_log


def export_csv(log, destination, columns=None, bounds=None):
    indices = [log.columns.index(name) for name in columns] if columns else list(range(len(log.columns)))
    left, right = 0, len(log.time)
    if bounds:
        left = int(np.searchsorted(log.time, bounds[0]))
        right = int(np.searchsorted(log.time, bounds[1], side="right"))
    with Path(destination).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["Time_elapsed(s)", *[log.columns[i] for i in indices]])
        for start in range(left, right, 8192):
            stop = min(start + 8192, right)
            for row, elapsed in zip(log.data[start:stop, indices], log.time[start:stop]):
                writer.writerow([format(elapsed, ".17g"), *[format(value, ".17g") for value in row]])


class Browser:
    def __init__(self, root, files=(), options=None):
        import tkinter as tk
        from tkinter import ttk
        from matplotlib import font_manager, rcParams
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        from matplotlib.figure import Figure

        available_fonts = {font.name for font in font_manager.fontManager.ttflist}
        preferred_fonts = ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei", "Arial Unicode MS")
        rcParams["font.sans-serif"] = [name for name in preferred_fonts if name in available_fonts] + ["DejaVu Sans"]

        self.root = root
        self.options = options or {}
        self.logs = {}
        self.signatures = {}
        self.entries = {}
        self.transforms = {}
        self.groups = [set()]
        self.axes = []
        self.lines = []
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="flight-log")
        self.events = queue.Queue()
        self.epoch = 0
        self.pending = 0
        self.syncing = False
        self.closed = False
        self.fft_job = None
        self.view_job = None
        self.play_job = None
        self.play_data = []
        root.title("CoFly Autopilot · Flight Data Browser")
        root.geometry("1450x900")
        root.minsize(1040, 700)
        root.protocol("WM_DELETE_WINDOW", self.close)

        pane = ttk.Panedwindow(root, orient="horizontal")
        pane.pack(fill="both", expand=True)
        left, right = ttk.Frame(pane, padding=10), ttk.Frame(pane)
        pane.add(left, weight=0)
        pane.add(right, weight=1)
        self.folder = tk.StringVar(value=str(Path(files[0]).resolve().parent if files else Path.cwd()))
        ttk.Label(left, text="数据目录").pack(anchor="w")
        folder_row = ttk.Frame(left)
        folder_row.pack(fill="x")
        ttk.Entry(folder_row, textvariable=self.folder, width=33).pack(side="left", fill="x", expand=True)
        ttk.Button(folder_row, text="浏览", command=self.browse_folder).pack(side="left")
        self.file_box = ttk.Combobox(left, state="readonly", width=40)
        self.file_box.pack(fill="x", pady=5)
        row = ttk.Frame(left)
        row.pack(fill="x")
        for label, callback in (("扫描", self.scan), ("加载所选", self.load_current),
                                ("打开文件…", self.open_files), ("清空", self.clear)):
            ttk.Button(row, text=label, command=callback).pack(side="left")
        settings = ttk.Frame(left)
        settings.pack(fill="x", pady=7)
        ttk.Label(settings, text="时间单位").pack(side="left")
        self.unit = tk.StringVar(value=self.options.get("time_unit", "auto"))
        ttk.Combobox(settings, state="readonly", values=("auto", "s", "ms", "us"),
                     textvariable=self.unit, width=6).pack(side="left")
        self.skip = tk.BooleanVar(value=self.options.get("skip_bad_rows", False))
        ttk.Checkbutton(settings, text="跳过坏行", variable=self.skip).pack(side="left")
        ttk.Label(left, text="Ctrl / Shift 多选变量；不同文件可叠加").pack(anchor="w")
        self.search = tk.StringVar()
        ttk.Entry(left, textvariable=self.search).pack(fill="x", pady=4)
        self.search.trace_add("write", lambda *_: self.refresh_tree())
        tree_frame = ttk.Frame(left)
        tree_frame.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(tree_frame, show="tree", selectmode="extended", height=16)
        scroll = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.tree.bind("<<TreeviewSelect>>", self.selection_changed)

        group_row = ttk.Frame(left)
        group_row.pack(fill="x", pady=5)
        self.group_box = ttk.Combobox(group_row, state="readonly", width=10)
        self.group_box.pack(side="left")
        self.group_box.bind("<<ComboboxSelected>>", self.change_group)
        ttk.Button(group_row, text="添加子图", command=self.add_group).pack(side="left")
        ttk.Button(group_row, text="清空子图", command=self.clear_group).pack(side="left")
        transform = ttk.Frame(left)
        transform.pack(fill="x")
        self.scale, self.offset = tk.StringVar(value="1"), tk.StringVar(value="0")
        for label, variable in (("缩放", self.scale), ("偏移", self.offset)):
            ttk.Label(transform, text=label).pack(side="left")
            ttk.Entry(transform, textvariable=variable, width=7).pack(side="left")
        ttk.Button(transform, text="应用", command=self.apply_transform).pack(side="left")
        ttk.Button(transform, text="恢复", command=self.reset_transform).pack(side="left")
        self.smooth = tk.BooleanVar(value=False)
        self.fft_enabled = tk.BooleanVar(value=False)
        toggles = ttk.Frame(left)
        toggles.pack(fill="x", pady=5)
        ttk.Checkbutton(toggles, text="显示平滑", variable=self.smooth, command=self.plot).pack(side="left")
        ttk.Checkbutton(toggles, text="区段 FFT", variable=self.fft_enabled, command=self.plot).pack(side="left")
        actions = ttk.Frame(left)
        actions.pack(fill="x")
        for label, callback in (("全时段", self.reset_view), ("飞行区段", self.zoom_air),
                                ("播放/停止", self.play), ("导出 CSV", self.export)):
            ttk.Button(actions, text=label, command=callback).pack(side="left")
        self.details = tk.Text(left, height=10, width=44, wrap="word", state="disabled")
        self.details.pack(fill="x", pady=5)
        self.status = tk.StringVar(value="打开一个或多个 CSV/TXT/LOG/DAT 文件。")
        ttk.Label(root, textvariable=self.status, anchor="w").pack(fill="x", padx=8, pady=4)

        self.figure = Figure(figsize=(10, 8), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.figure, master=right)
        NavigationToolbar2Tk(self.canvas, right).pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.update_groups()
        self.scan()
        self.poll_job = root.after(100, self.poll)
        for file in files:
            self.load(file)

    def show_details(self, text):
        self.details.configure(state="normal")
        self.details.delete("1.0", "end")
        self.details.insert("1.0", text)
        self.details.configure(state="disabled")

    def browse_folder(self):
        from tkinter import filedialog
        folder = filedialog.askdirectory(initialdir=self.folder.get())
        if folder:
            self.folder.set(folder)
            self.scan()

    def scan(self):
        folder = Path(self.folder.get()).expanduser()
        try:
            self.files = sorted((p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in
                                 (".csv", ".txt", ".log", ".dat")), key=lambda p: p.name.casefold())
            self.file_box["values"] = [p.name for p in self.files]
            self.file_box.set(self.files[0].name if self.files else "")
        except OSError as exc:
            self.files = []
            self.status.set(str(exc))

    def open_files(self):
        from tkinter import filedialog
        for file in filedialog.askopenfilenames(initialdir=self.folder.get(),
                                               filetypes=[("Flight logs", "*.txt *.csv *.log *.dat"), ("All", "*.*")]):
            self.load(file)

    def load_current(self):
        index = self.file_box.current()
        if 0 <= index < len(self.files):
            self.load(self.files[index])

    def load(self, file):
        path = Path(file).expanduser().resolve()
        options = dict(self.options, time_unit=self.unit.get(), skip_bad_rows=self.skip.get())
        try:
            stat = path.stat()
        except OSError as exc:
            self.status.set(str(exc))
            return
        signature = (stat.st_size, stat.st_mtime_ns, tuple(sorted(options.items())))
        if self.signatures.get(path) == signature:
            self.status.set(f"已加载，直接复用内存：{path.name}")
            return
        self.pending += 1
        self.status.set(f"后台读取 {path.name}；待完成 {self.pending} 个文件…")
        epoch = self.epoch

        def work():
            try:
                result = load_log(path, **options)
                self.events.put((epoch, path, signature, result, None))
            except Exception as exc:
                self.events.put((epoch, path, signature, None, str(exc)))
        self.executor.submit(work)

    def poll(self):
        from tkinter import messagebox
        while not self.events.empty():
            epoch, path, signature, log, error = self.events.get_nowait()
            if epoch != self.epoch:
                continue
            self.pending -= 1
            if error:
                self.status.set(f"加载失败：{path.name}")
                messagebox.showerror("日志读取失败", f"{path}\n\n{error}", parent=self.root)
                continue
            self.logs[path] = log
            self.signatures[path] = signature
            self.refresh_tree()
            if not any(self.groups):
                self.groups[0] = {(path, i) for i in range(1, min(4, len(log.columns)))}
                self.select_group()
            self.show_details(json.dumps(log.summary(), ensure_ascii=False, indent=2))
            self.plot()
            self.status.set(f"{path.name} · {len(log.time):,} 行 × {len(log.columns)} 列 · "
                            f"{log.sample_rate:.3f} Hz · {log.data.nbytes / 2**20:.1f} MiB"
                            f" · {len(log.warnings)} 条提示 · 待完成 {self.pending}")
        if not self.closed:
            self.poll_job = self.root.after(100, self.poll)

    def refresh_tree(self):
        self.syncing = True
        self.tree.delete(*self.tree.get_children())
        self.entries.clear()
        query = self.search.get().casefold()
        for log_number, (path, log) in enumerate(self.logs.items()):
            parent = self.tree.insert("", "end", text=f"{path.parent.name}/{path.name}", open=True)
            for i, column in enumerate(log.columns):
                if query and query not in column.casefold():
                    continue
                item = self.tree.insert(parent, "end", iid=f"{log_number}:{i}", text=column)
                self.entries[item] = (path, i)
        self.select_group()
        self.syncing = False

    def selected(self):
        return {self.entries[item] for item in self.tree.selection() if item in self.entries}

    def select_group(self):
        index = max(self.group_box.current(), 0)
        group = self.groups[index]
        self.syncing = True
        self.tree.selection_set([item for item, key in self.entries.items() if key in group])
        self.syncing = False

    def selection_changed(self, _event=None):
        if self.syncing:
            return
        index = max(self.group_box.current(), 0)
        # A filtered tree hides entries; keep these assigned to the subplot.
        visible = set(self.entries.values())
        self.groups[index] = (self.groups[index] - visible) | self.selected()
        text = []
        for path, column in sorted(self.selected()):
            log = self.logs[path]
            scale, offset = self.transforms.get((path, column), (1, 0))
            text.append(f"{path.parent.name}/{path.name} | {log.columns[column]}\n"
                        f"{describe_variable(log.columns[column])}\ny = 原值 × {scale:g} + {offset:g}")
        self.show_details("\n\n".join(text))
        self.plot()

    def update_groups(self):
        self.group_box["values"] = [f"子图 {i + 1}" for i in range(len(self.groups))]
        self.group_box.current(len(self.groups) - 1)

    def change_group(self, _event=None):
        self.select_group()
        self.update_fft()

    def add_group(self):
        if len(self.groups) >= 8:
            self.status.set("最多显示 8 个子图。")
            return
        self.groups.append(set())
        self.update_groups()
        self.select_group()
        self.plot()

    def clear_group(self):
        self.groups[max(0, self.group_box.current())].clear()
        self.select_group()
        self.plot()

    def apply_transform(self):
        try:
            scale, offset = float(self.scale.get()), float(self.offset.get())
            if not np.isfinite([scale, offset]).all():
                raise ValueError()
        except ValueError:
            self.status.set("缩放与偏移必须是有限数值。")
            return
        for entry in self.selected():
            self.transforms[entry] = (scale, offset)
        self.selection_changed()

    def reset_transform(self):
        for entry in self.selected():
            self.transforms.pop(entry, None)
        self.scale.set("1")
        self.offset.set("0")
        self.selection_changed()

    def series(self, entry):
        path, column = entry
        log = self.logs[path]
        scale, offset = self.transforms.get(entry, (1, 0))
        values = log.data[:, column]
        if scale != 1 or offset != 0:
            values = values * scale + offset
        return log, values

    def plot(self):
        self.stop_play()
        old_view = self.axes[0].get_xlim() if self.axes and self.lines else None
        if self.view_job:
            self.root.after_cancel(self.view_job)
            self.view_job = None
        self.figure.clear()
        self.axes, self.lines = [], []
        grid = self.figure.add_gridspec(len(self.groups) + int(self.fft_enabled.get()), 1)
        for group_index, group in enumerate(self.groups):
            ax = self.figure.add_subplot(grid[group_index], sharex=self.axes[0] if self.axes else None)
            self.axes.append(ax)
            ax.grid(True, alpha=0.25)
            ax.set_ylabel(f"S{group_index + 1}")
            references = set()
            for entry in sorted(group):
                if entry[0] not in self.logs or entry[1] >= len(self.logs[entry[0]].columns):
                    continue
                log, values = self.series(entry)
                width = max(1, round(log.sample_rate * 0.2)) if self.smooth.get() else 1
                x, y = display_series(log.time, values, smooth_samples=width)
                label = f"{log.path.parent.name}/{log.path.name} | {log.columns[entry[1]]}"
                line, = ax.plot(x, y, linewidth=0.9, label=label)
                self.lines.append((line, entry))
                if log.path not in references:
                    references.add(log.path)
                    for segment in log.in_air:
                        ax.axvspan(segment.start, segment.stop, color="#ffd369", alpha=0.13)
            if references:
                # Mode shading uses one explicit reference rather than conflicting
                # overlays from independently normalized aircraft clocks.
                ref = self.logs[sorted(references)[0]]
                colors = {"Manual": "#ec8f81", "Stab1": "#81b8ec", "Stab2": "#8cca8c", "Unknown": "#aaaaaa"}
                for segment in ref.modes:
                    ax.axvspan(segment.start, segment.stop, ymin=0.94, ymax=1,
                               color=colors[segment.label], alpha=0.7)
                ax.set_title(f"Mode reference: {ref.path.parent.name}/{ref.path.name}", fontsize=9)
                ax.legend(fontsize=7, loc="upper right")
        self.axes[-1].set_xlabel("Time from first valid sample (s)")
        if old_view:
            self.axes[0].set_xlim(old_view)
        if self.fft_enabled.get():
            self.fft_ax = self.figure.add_subplot(grid[-1])
            self.update_fft()
        else:
            self.fft_ax = None
        self.figure.tight_layout()
        for ax in self.axes:
            ax.callbacks.connect("xlim_changed", self.view_changed)
        self.canvas.draw_idle()

    def view_changed(self, _ax):
        if self.view_job:
            self.root.after_cancel(self.view_job)
        self.view_job = self.root.after(180, self.refresh_view)

    def refresh_view(self):
        self.view_job = None
        if not self.axes:
            return
        bounds = self.axes[0].get_xlim()
        for line, entry in self.lines:
            log, values = self.series(entry)
            left = max(0, int(np.searchsorted(log.time, bounds[0])) - 1)
            right = min(len(log.time), int(np.searchsorted(log.time, bounds[1], side="right")) + 1)
            width = max(1, round(log.sample_rate * 0.2)) if self.smooth.get() else 1
            line.set_data(*display_series(log.time[left:right], values[left:right], smooth_samples=width))
        self.update_fft()
        self.canvas.draw_idle()

    def update_fft(self):
        if not self.fft_enabled.get() or not hasattr(self, "fft_ax") or self.fft_ax is None or not self.axes:
            return
        ax = self.fft_ax
        ax.clear()
        notes = []
        for entry in sorted(self.groups[max(0, self.group_box.current())]):
            if entry[0] not in self.logs:
                continue
            log, values = self.series(entry)
            label = f"{log.path.parent.name}/{log.path.name}:{log.columns[entry[1]]}"
            try:
                spectrum = compute_spectrum(log.time, values, self.axes[0].get_xlim())
                ax.plot(spectrum.frequency, spectrum.amplitude, label=label)
                notes.append(f"{label}: {spectrum.peak_frequency:.3g} Hz / {spectrum.peak_amplitude:.3g}"
                             + (" (resampled)" if spectrum.resampled else ""))
            except LogError as exc:
                notes.append(f"{label}: {exc}")
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Amplitude")
        ax.grid(True, alpha=0.25)
        if ax.lines:
            ax.legend(fontsize=7)
        self.status.set(" | ".join(notes) or "当前子图未选择 FFT 变量。")
        self.canvas.draw_idle()

    def reset_view(self):
        if self.logs and self.axes:
            self.axes[0].set_xlim(0, max(log.time[-1] for log in self.logs.values()))

    def zoom_air(self):
        entries = sorted(self.groups[max(0, self.group_box.current())])
        if entries and self.axes:
            log = self.logs[entries[0][0]]
            if log.in_air:
                segment = max(log.in_air, key=lambda s: s.stop - s.start)
                self.axes[0].set_xlim(max(0, segment.start - 2), min(log.time[-1], segment.stop + 2))
            else:
                self.status.set("所选文件没有识别出飞行区段；估算依据为空速/绝对油门 PWM。")

    def play(self):
        if self.play_job:
            self.stop_play()
            return
        if not self.lines:
            return
        self.play_data = [(line, np.asarray(line.get_xdata()), np.asarray(line.get_ydata())) for line, _ in self.lines]
        start, stop = self.axes[0].get_xlim()
        self.frames = iter(np.linspace(start, stop, 150))
        self.play_step()

    def play_step(self):
        current = next(self.frames, None)
        if current is None:
            self.stop_play()
            return
        for line, x, y in self.play_data:
            end = np.searchsorted(x, current, side="right")
            line.set_data(x[:end], y[:end])
        self.canvas.draw_idle()
        self.play_job = self.root.after(33, self.play_step)

    def stop_play(self):
        if self.play_job:
            self.root.after_cancel(self.play_job)
            self.play_job = None
        for line, x, y in self.play_data:
            line.set_data(x, y)
        self.play_data = []

    def export(self):
        from tkinter import filedialog, messagebox
        selected = sorted(self.selected())
        paths = {entry[0] for entry in selected}
        if len(paths) != 1:
            self.status.set("导出时请选择同一个文件的变量；不同文件时间轴独立。")
            return
        log = self.logs[selected[0][0]]
        destination = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=log.path.stem + "_selected.csv")
        if destination:
            if Path(destination).resolve() in self.logs:
                messagebox.showerror("导出失败", "不能覆盖已加载的原始日志。", parent=self.root)
                return
            try:
                export_csv(log, destination, [log.columns[i] for _, i in selected], self.axes[0].get_xlim())
            except (OSError, ValueError) as exc:
                messagebox.showerror("导出失败", str(exc), parent=self.root)
                return
            self.status.set(f"已导出原始数据（不应用显示平滑/变换）：{destination}")

    def clear(self):
        self.stop_play()
        self.epoch += 1
        self.pending = 0
        self.logs.clear()
        self.signatures.clear()
        self.transforms.clear()
        self.groups = [set()]
        self.update_groups()
        self.refresh_tree()
        self.plot()

    def close(self):
        self.closed = True
        self.stop_play()
        for job in (self.poll_job, self.view_job, self.fft_job):
            if job:
                self.root.after_cancel(job)
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.root.destroy()


def main():
    # Keep Chinese diagnostics and redirected JSON readable on Windows too.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="CoFly 飞行日志浏览、摘要、区段 FFT 与 CSV 导出")
    parser.add_argument("files", nargs="*", type=Path)
    parser.add_argument("--summary", action="store_true", help="输出 JSON 摘要，不打开窗口")
    parser.add_argument("--export", type=Path, help="导出一个日志的原始数据为 CSV")
    parser.add_argument("--columns", nargs="+", help="导出列名（包含括号时请加引号）")
    parser.add_argument("--fft", help="输出指定列在所选区段的频谱峰值")
    parser.add_argument("--from", dest="start", type=float, default=0)
    parser.add_argument("--to", dest="stop", type=float, default=float("inf"))
    parser.add_argument("--time-column")
    parser.add_argument("--time-unit", choices=("auto", "s", "ms", "us"), default="auto")
    parser.add_argument("--fallback-hz", type=float, default=50)
    parser.add_argument("--encoding", default="utf-8-sig")
    parser.add_argument("--delimiter", help="默认识别逗号、制表符或分号")
    parser.add_argument("--skip-bad-rows", action="store_true")
    args = parser.parse_args()
    options = {key: getattr(args, key) for key in ("time_column", "time_unit", "fallback_hz", "encoding", "delimiter", "skip_bad_rows")}
    if args.summary or args.export or args.fft:
        if not args.files:
            parser.error("命令行分析需要至少一个文件。")
        if args.export and len(args.files) != 1:
            parser.error("一次导出一个文件，避免不同时间轴被拼接。")
        if not np.isfinite(args.start) or np.isnan(args.stop) or args.start >= args.stop:
            parser.error("时间区段必须满足 --from < --to。")
        results = []
        try:
            for file in args.files:
                log = load_log(file, **options)
                summary = log.summary()
                if args.fft:
                    spectrum = compute_spectrum(log.time, log.column(args.fft), (args.start, args.stop))
                    summary["fft"] = dict(column=args.fft, peak_hz=spectrum.peak_frequency,
                                          peak_amplitude=spectrum.peak_amplitude, samples=spectrum.samples,
                                          resampled=spectrum.resampled)
                if args.export:
                    if args.export.resolve() == log.path:
                        raise LogError("导出路径不能覆盖原始日志。")
                    export_csv(log, args.export, args.columns, (args.start, args.stop))
                    summary["export"] = str(args.export)
                results.append(summary)
        except (OSError, ValueError, UnicodeError) as exc:
            parser.exit(2, f"读取失败：{exc}\n")
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        import tkinter as tk
        root = tk.Tk()
        Browser(root, args.files, options)
        root.mainloop()


if __name__ == "__main__":
    main()
