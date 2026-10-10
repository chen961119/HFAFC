"""Serial I/O runs outside Tk's UI thread; events carry a connection identity."""
import queue
import threading
import serial


class SerialWorker(threading.Thread):
    def __init__(self, port, baudrate, events, connection_id):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.events = events
        self.connection_id = connection_id
        self.commands = queue.Queue()
        self.stopping = threading.Event()

    def emit(self, kind, payload=None):
        self.events.put((self.connection_id, kind, payload))

    def send(self, data):
        self.commands.put(data)

    def stop(self):
        self.stopping.set()

    def run(self):
        try:
            with serial.serial_for_url(self.port, baudrate=self.baudrate,
                                       timeout=0.05, write_timeout=0.5) as connection:
                if self.stopping.is_set():
                    return
                self.emit("connected")
                while not self.stopping.is_set():
                    try:
                        data = self.commands.get_nowait()
                    except queue.Empty:
                        data = None
                    if data is not None:
                        if connection.write(data) != len(data):
                            raise serial.SerialException("串口未完整发送命令")
                        self.emit("tx", data)
                    chunk = connection.read(min(max(connection.in_waiting, 1), 8192))
                    if chunk:
                        self.emit("rx", chunk)
        except (serial.SerialException, OSError, ValueError) as exc:
            if not self.stopping.is_set():
                self.emit("error", str(exc))
        finally:
            self.emit("closed")
