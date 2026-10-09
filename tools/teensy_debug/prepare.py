"""Build/upload matching firmware and locate Teensy's single USB serial port."""
import argparse
import os
from pathlib import Path
import shutil
import stat
import subprocess
import time
import sys

from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[2]


def repair_teensydebug_cache():
    """Discard an incomplete generated package so PlatformIO can reinstall it."""
    package = ROOT / ".pio" / "libdeps" / "teensy41_debug" / "TeensyDebug"
    if not package.is_dir():
        return
    if any((package / name).is_file() for name in
           ("library.json", "library.properties", "module.json")):
        return
    # Never follow a junction/symlink or remove a directory outside this project.
    resolved_package = package.resolve()
    if (package.is_symlink() or ROOT.resolve() not in resolved_package.parents
            or resolved_package.parent != package.parent.resolve()):
        raise RuntimeError(f"Unsafe TeensyDebug cache path: {package}")
    print("Removing incomplete TeensyDebug install from .pio/libdeps.", flush=True)
    def clear_readonly(function, path, _error):
        os.chmod(path, stat.S_IWRITE)
        function(path)

    shutil.rmtree(package, onerror=clear_readonly)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--attach", action="store_true")
    args = parser.parse_args()
    if not args.attach:
        repair_teensydebug_cache()
        pio = shutil.which("platformio") or str(
            Path.home() / ".platformio/penv/Scripts/platformio.exe"
        )
        process = subprocess.Popen(
            [pio, "run", "-e", "teensy41_debug", "-t", "upload"],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, errors="replace")
        missing_board = False
        for line in process.stdout:
            print(line, end="", flush=True)
            missing_board |= "No Teensy boards were found" in line
        if process.wait():
            raise RuntimeError("Firmware build/upload failed.")
        if missing_board:
            print("Uploader is waiting: press PROGRAM on the connected Teensy.", flush=True)
    if not (ROOT / ".pio/build/teensy41_debug/firmware.elf").is_file():
        raise RuntimeError("Debug ELF not found: build/upload the debug environment first.")
    deadline = time.monotonic() + 20
    while True:
        ports = [p for p in list_ports.comports()
                 if p.vid == 0x16C0 and p.pid == 0x0483]
        debug_ports = ports
        if len(debug_ports) > 1:
            raise RuntimeError("Multiple Teensy debug ports: connect only one board.")
        if debug_ports:
            port = debug_ports[0]
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("Teensy USB serial port not found. Check USB/upload; "
                               "PlatformIO success alone does not confirm board upload.")
        time.sleep(0.2)
    output = ROOT / ".pio/teensydebug"
    output.mkdir(parents=True, exist_ok=True)
    (output / "connect.gdb").write_text(
        "set remotetimeout 10\n"
        "set mem inaccessible-by-default off\n"
        f"target extended-remote \\\\.\\{port.device}\n", encoding="utf-8")
    print(f"TeensyDebug: {port.device}")
    print("Debug firmware: this USB port is reserved for GDB; parameter/console traffic is disabled.")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError) as error:
        print(f"TeensyDebug: {error}", file=sys.stderr)
        sys.exit(1)
