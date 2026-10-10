"""Hidden-window integration test for async loading and plotting."""
from pathlib import Path
import tempfile
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from main import Browser


class BrowserTests(unittest.TestCase):
    def test_async_comparison_transform_filter_fft_and_empty_view(self):
        with tempfile.TemporaryDirectory() as folder:
            files = []
            for number in range(2):
                path = Path(folder) / f"log{number}.csv"
                rows = "\n".join(f"{i * 20000},{i % 8},1500" for i in range(400))
                path.write_text("TimeStamp(us),signal,CH5_PWM\n" + rows)
                files.append(path)
            root = tk.Tk()
            root.withdraw()
            with patch("tkinter.messagebox.showerror", side_effect=AssertionError):
                browser = Browser(root, files)
                try:
                    deadline = time.monotonic() + 10
                    while len(browser.logs) != 2 and time.monotonic() < deadline:
                        root.update()
                        time.sleep(.01)
                    self.assertEqual(len(browser.logs), 2)
                    entries = {(path.resolve(), 1) for path in files}
                    browser.groups[0] = entries
                    browser.select_group()
                    browser.plot()
                    self.assertEqual(len(browser.lines), 2)
                    browser.scale.set("2")
                    browser.offset.set("-3")
                    browser.apply_transform()
                    self.assertEqual(set(browser.transforms.values()), {(2, -3)})
                    browser.search.set("not-present")
                    root.update()
                    self.assertEqual(browser.groups[0], entries)
                    browser.search.set("")
                    root.update()
                    browser.fft_enabled.set(True)
                    browser.plot()
                    self.assertEqual(len(browser.fft_ax.lines), 2)
                    browser.add_group()
                    self.assertEqual(len(browser.axes), 2)
                    browser.smooth.set(True)
                    browser.axes[0].set_xlim(100, 101)
                    browser.refresh_view()
                    browser.play()
                    browser.stop_play()
                    browser.clear()
                    self.assertFalse(browser.logs)
                finally:
                    browser.close()


if __name__ == "__main__":
    unittest.main()
