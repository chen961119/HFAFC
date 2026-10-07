"""Line framing for parameter replies mixed with ordinary USB serial output."""
from dataclasses import dataclass
from decimal import Decimal
import math
import struct
import re


@dataclass(frozen=True)
class Parameter:
    name: str
    value: float
    minimum: float
    maximum: float
    group: str
    description: str
    dtype: str = "float"


def format_value(value, dtype="float"):
    if dtype == "int":
        return str(int(value))
    bits = struct.pack("<f", value)
    number = struct.unpack("<f", bits)[0]
    for digits in range(1, 10):
        text = format(number, f".{digits}g")
        try:
            matches = struct.pack("<f", float(text)) == bits
        except OverflowError:
            matches = False
        if matches:
            return format(Decimal(text), "f")
    return format(Decimal(text), "f")


def parse_integer(text):
    if not re.fullmatch(r"[+-]?[0-9]+", text):
        raise ValueError("整型参数请输入十进制整数")
    number = int(text)
    if not -2147483648 <= number <= 2147483647:
        raise ValueError("数值超过 int32 的范围")
    return number


class ReplyFramer:
    def __init__(self):
        self.buffer = bytearray()
        self.discarding = False

    def feed(self, data: bytes):
        replies = []
        for byte in data:
            if byte == 10:
                if not self.discarding:
                    reply = parse_reply(bytes(self.buffer).rstrip(b"\r"))
                    if reply is not None:
                        replies.append(reply)
                self.buffer.clear()
                self.discarding = False
            elif not self.discarding:
                if len(self.buffer) >= 1024:
                    self.buffer.clear()
                    self.discarding = True
                else:
                    self.buffer.append(byte)
        return replies


def parse_reply(line: bytes):
    if not line.startswith(b"@HFAFC\t"):
        return None
    try:
        fields = line.decode("ascii").split("\t")
        if len(fields) < 3 or not fields[1].isascii() or not fields[1].isdigit():
            return None
        request_id = int(fields[1])
        if not 0 <= request_id <= 0xFFFFFFFF:
            return None
        return request_id, fields[2], fields[3:]
    except UnicodeDecodeError:
        return None


def parameter_from_fields(fields):
    dtype = "float"
    if len(fields) == 7:
        name, dtype, *remaining = fields
        fields = [name, *remaining]
    if len(fields) != 6 or dtype not in ("float", "int"):
        raise ValueError("参数响应字段数量错误")
    name, value, low, high, group, description = fields
    convert = parse_integer if dtype == "int" else float
    numbers = tuple(convert(item) for item in (value, low, high))
    if not name or not all(math.isfinite(item) for item in numbers):
        raise ValueError("参数响应包含无效数值")
    if not numbers[1] <= numbers[0] <= numbers[2]:
        raise ValueError("参数值超出返回的范围")
    return Parameter(name, *numbers, group, description, dtype)


def validate_edit(parameter: Parameter, text: str) -> float:
    if parameter.dtype == "int":
        value = parse_integer(text)
        if parameter.name.endswith("_rev") and value not in (-1, 1):
            raise ValueError("反向系数只能为 -1 或 1")
        if not parameter.minimum <= value <= parameter.maximum:
            raise ValueError(f"允许范围：{format_value(parameter.minimum, parameter.dtype)} ～ {format_value(parameter.maximum, parameter.dtype)}")
        return value
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError("请输入有效数字") from exc
    if not math.isfinite(value):
        raise ValueError("请输入有限数值")
    # Validate using the controller's float32 representation, including limits
    # such as -0.001 whose nine-digit wire representation is slightly different.
    try:
        as_float32 = lambda number: struct.unpack("<f", struct.pack("<f", number))[0]
        value = as_float32(value)
        minimum = as_float32(parameter.minimum)
        maximum = as_float32(parameter.maximum)
    except (OverflowError, struct.error) as exc:
        raise ValueError("数值超过飞控 float32 的范围") from exc
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"允许范围：{format_value(parameter.minimum)} ～ {format_value(parameter.maximum)}")
    return value
