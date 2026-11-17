from dataclasses import dataclass, field
from enum import IntEnum
from typing import Self, Final, Any, Literal
import struct


class ProtocolError(ValueError):
    ...


@dataclass(frozen=True, slots=True)
class Format:
    code: str
    fields: tuple[str, ...]
    codec: struct.Struct | None = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not self.code:
            if self.fields:
                raise ProtocolError("Empty format code cannot define fields.")
            object.__setattr__(self, "codec", None)
            return

        not_implemented = {"p", "s"}
        if set(self.code) & not_implemented:
            raise NotImplementedError(f"Format code doesn't support {' and '.join(not_implemented)} array types.")

        stripped = self.code.lstrip("<>!@=")
        full_code = ""
        digits = ""
        for char in stripped:
            if char.isdecimal():
                digits += char
            else:
                full_code += char * (int(digits) if digits else 1)
                digits = ""

        if len(full_code) != len(self.fields):
            raise ProtocolError(f"Format code '{self.code}' doesn't match number of fields ({len(self.fields)}).")

        object.__setattr__(self, "codec", struct.Struct(self.code))

    @property
    def size(self) -> int:
        return self.codec.size if self.codec else 0


# Reusable command building blocks
EMPTY = Format("", ())
BYTE_ARRAY_SIZE: Final[int] = 16
BYTE_ARRAY = Format(F">{BYTE_ARRAY_SIZE}H", tuple(f"field_{i}" for i in range(BYTE_ARRAY_SIZE)))
AXIS_ID = Format(">B", ("axis_id",))
AXIS_TARGET = Format(">Bf", ("axis_id", "position"))
RESULT = Format(">H", ("result",))
POSITION_F = Format(">f", ("position",))
LOG = Format("<q5f3i5f", ("timestamp_us", "current", "voltage", "angle", "position",         # This is the only time the endianness
                          "target_position", "step_position", "commanded_hz", "applied_hz",  # differs from other frames
                          "x_hat1", "x_hat2", "x_hat3", "ref_position", "ref_velocity"))


class Command(IntEnum):
    rx: Format | None
    tx: Format | None

    def __new__(cls, opcode: int, tx: Format | None, rx: Format | None) -> Self:
        obj: Self = int.__new__(cls, opcode)
        obj._value_ = opcode
        obj.tx = tx
        obj.rx = rx
        return obj

    ECHO                 = (0x01, BYTE_ARRAY, BYTE_ARRAY)
    CLEAR_RX             = (0x02, EMPTY, EMPTY)
    STEP_ARM             = (0x03, AXIS_ID, RESULT)
    STEP_DISARM          = (0x04, AXIS_ID, RESULT)
    STEP_GET_POS         = (0x08, AXIS_ID, Format(">l", ("position",)))
    SENSE_GET_POS        = (0x09, AXIS_ID, POSITION_F)
    SENSE_GET_PWR        = (0x0B, AXIS_ID, Format(">f", ("power",)))
    SENSE_GET_CURR       = (0x0C, AXIS_ID, Format(">f", ("current",)))
    SENSE_GET_VOLT       = (0x0D, AXIS_ID, Format(">f", ("voltage",)))
    MCTRL_MOVE           = (0x0E, AXIS_TARGET, RESULT)
    MCTRL_STOP           = (0x12, AXIS_ID, RESULT)
    MCTRL_GET_POS        = (0x0F, AXIS_ID, POSITION_F)
    MCTRL_ERROR          = (0x18, None, AXIS_ID)
    MCTRL_RESET_ERROR    = (0x14, AXIS_ID, RESULT)
    MCTRL_LOG_COUNT      = (0x11, None, Format(">l", ("count",)))
    MCTRL_DUMP_LOG       = (0x10, AXIS_ID, LOG)
    SERVO_SET_POS        = (0x15, POSITION_F, RESULT)
    SERVO_GET_POS        = (0x16, EMPTY, POSITION_F)
    SERVO_TRIGGER        = (0x17, EMPTY, Format(">HH", ("result_1", "result_2")))


def encode(cmd: Command, **kwargs: Any) -> bytes:
    fmt = cmd.tx
    if fmt is None:
        raise ProtocolError(f"Cannot encode cmd {cmd.name} (tx is None).")

    expected = set(fmt.fields)
    supplied = set(kwargs)
    if missing := expected - supplied:
        raise ProtocolError(f"{cmd.name} missing fields: {sorted(missing)}")
    if extra := supplied - expected:
        raise ProtocolError(f"{cmd.name} got unexpected fields: {sorted(extra)}")

    if fmt.codec is None:
        data_bytes = b""
    else:
        ordered_values = [kwargs[name] for name in fmt.fields]
        try:
            data_bytes = fmt.codec.pack(*ordered_values)
        except struct.error as exc:
            raise ProtocolError(f"Unable to pack {cmd.name}: {exc}") from exc

    header_bytes = bytes([cmd, fmt.size])
    checksum_bytes = bytes([sum(header_bytes + data_bytes) % 256])

    return header_bytes + checksum_bytes + data_bytes


def decode(cmd: Command, full_payload: bytes) -> dict[str, Any]:
    fmt = cmd.rx
    if fmt is None:
        raise ProtocolError(f"Cannot decode cmd {cmd.name} (rx is None).")

    if len(full_payload) < 3:
        raise ProtocolError(f"Frame too short ({len(full_payload)} bytes); expected at least 3.")

    try:
        payload_cmd = Command(full_payload[0])  # type: ignore[call-arg]
    except ValueError as exc:
        raise ProtocolError(f"Unknown cmd opcode 0x{full_payload[0]:02X}.") from exc

    if payload_cmd != cmd:
        raise ProtocolError(f"Expected {cmd.name}, got {payload_cmd.name}.")

    declared_len = full_payload[1]
    body = full_payload[3:]
    if declared_len != len(body):
        raise ProtocolError(f"Declared payload length ({declared_len}) does not match actual ({len(body)}).")

    expected_checksum = sum(full_payload[:2] + body) % 256
    actual_checksum = full_payload[2]
    if expected_checksum != actual_checksum:
        raise ProtocolError(f"Checksum mismatch: expected {expected_checksum}, got {actual_checksum}.")

    if fmt.codec is None:
        return {}

    if len(body) != fmt.size:
        raise ProtocolError(f"{cmd.name} expects {fmt.size} data bytes, got {len(body)}.")

    return dict(zip(fmt.fields, fmt.codec.unpack(body)))
