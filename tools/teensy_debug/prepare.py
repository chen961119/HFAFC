"""Build/upload matching firmware and identify both Teensy USB CDC interfaces."""
import argparse
import os
from pathlib import Path
import shutil
import stat
import subprocess
import time
import sys
import re

from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[2]


def usb_interface(port):
    """Use interface metadata, never COM ordering (CDC0=ground, CDC2=GDB)."""
    match = re.search(r"MI_([0-9a-f]{2})", getattr(port, "hwid", "") or "", re.I)
    if match:
        return int(match[1], 16)
    # pyserial: Windows LOCATION=1-2:x.2; Linux LOCATION=1-2:1.2.
    match = re.search(r":(?:x|\d+)\.(\d+)$", getattr(port, "location", "") or "", re.I)
    return int(match[1]) if match else None


def select_usb_ports(ports):
    ports = [p for p in ports if p.vid == 0x16C0 and p.pid == 0x048B]
    if len(ports) > 2:
        raise RuntimeError("Multiple dual-serial Teensy boards: connect only one board.")
    ground = [p for p in ports if usb_interface(p) == 0]
    debug = [p for p in ports if usb_interface(p) == 2]
    if len(ground) != 1 or len(debug) != 1:
        return None
    ground, debug = ground[0], debug[0]
    for attribute in ("serial_number", "location"):
        first = getattr(ground, attribute, None)
        second = getattr(debug, attribute, None)
        if attribute == "location":
            first = first.split(":")[0] if first else None
            second = second.split(":")[0] if second else None
        if first and second and first != second:
            raise RuntimeError("Ground station and GDB ports belong to different Teensy boards.")
    return ground, debug


def repair_library_cache():
    """Discard incomplete project dependencies before PlatformIO scans them."""
    cache = ROOT / ".pio" / "libdeps" / "teensy41_debug"
    if not cache.is_dir():
        return
    # The failed package can be any direct or transitive library, not just TeensyDebug.
    manifests = ("library.json", "library.properties", "module.json")
    def clear_readonly(function, path, _error):
        os.chmod(path, stat.S_IWRITE)
        function(path)

    for package in cache.iterdir():
        if not package.is_dir() or any((package / name).is_file() for name in manifests):
            continue
        # Never follow a junction/symlink or remove a directory outside this project.
        resolved_package = package.resolve()
        if (package.is_symlink() or ROOT.resolve() not in resolved_package.parents
                or resolved_package.parent != cache.resolve()):
            raise RuntimeError(f"Unsafe library cache path: {package}")
        print(f"Removing incomplete PlatformIO library: {package.name}", flush=True)
        shutil.rmtree(package, onerror=clear_readonly)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--attach", action="store_true")
    args = parser.parse_args()
    if not args.attach:
        vendor_manifest = ROOT / "tools/teensy_debug/vendor/TeensyDebug/library.properties"
        if not vendor_manifest.is_file():
            raise RuntimeError(f"Vendored TeensyDebug manifest is missing: {vendor_manifest}")
        repair_library_cache()
        pio = shutil.which("platformio") or str(
            Path.home() / ".platformio/penv/Scripts/platformio.exe"
        )
        process = subprocess.Popen(
            [pio, "run", "-e", "teensy41_debug", "-t", "upload"],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, errors="replace")
        missing_board = False
        missing_manifest = False
        for line in process.stdout:
            print(line, end="", flush=True)
            missing_board |= "No Teensy boards were found" in line
            missing_manifest |= "MissingPackageManifestError" in line
        if process.wait():
            if missing_manifest:
                raise RuntimeError("PlatformIO still found a library without a manifest. "
                                   "Check the library name printed by the cache repair step "
                                   "and share the full build output.")
            raise RuntimeError("Firmware build/upload failed.")
        if missing_board:
            print("Uploader is waiting: press PROGRAM on the connected Teensy.", flush=True)
    if not (ROOT / ".pio/build/teensy41_debug/firmware.elf").is_file():
        raise RuntimeError("Debug ELF not found: build/upload the debug environment first.")
    deadline = time.monotonic() + 20
    while True:
        selected = select_usb_ports(list_ports.comports())
        if selected:
            ground_port, port = selected
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("Teensy dual USB serial ports (VID 16C0 / PID 048B, interfaces 0 and 2) not found. "
                               "Upload the new debug firmware and check USB enumeration/interface metadata; "
                               "PlatformIO success alone does not confirm board upload.")
        time.sleep(0.2)
    output = ROOT / ".pio/teensydebug"
    output.mkdir(parents=True, exist_ok=True)
    (output / "connect.gdb").write_text(
        "set remotetimeout 10\n"
        "set mem inaccessible-by-default off\n"
        f"target extended-remote \\\\.\\{port.device}\n", encoding="utf-8")
    print(f"TeensyDebug: {port.device}")
    print(f"Ground station / USB console: {ground_port.device}")
    print("Debug firmware: two virtual serial ports on one USB cable. "
          "Use the ground station port for parameters/console, and the debug port for GDB.")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError) as error:
        print(f"TeensyDebug: {error}", file=sys.stderr)
        sys.exit(1)
