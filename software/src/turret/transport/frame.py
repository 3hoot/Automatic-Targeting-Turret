import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import NamedTuple, Any


MAX_FRAME_DATA_LENGTH = 128
MIN_FRAME_DATA_LENGTH = 3


@dataclass(slots=True)
class Format:
    byte_format: str
    fields: tuple[str, ...]
    is_array: bool = False


class Spec(NamedTuple):
    TX: Format
    RX: Format


class Type(IntEnum):
    ECHO           = 0x01
    CLEAR_RX       = 0x02
    STEP_ARM       = 0x03
    STEP_DISARM    = 0x04
    GET_RAW_POS    = 0x09
    MOVE           = 0x0E
    GET_POS        = 0x0F
    LOG            = 0x10
    RESET_WATCHDOG = 0x14


SCHEMAS: dict[Type, Spec] = {
    Type.ECHO: Spec(
        TX=Format(">B", ("data",), is_array=True),
        RX=Format(">B", ("data",), is_array=True), 
    ),
    Type.CLEAR_RX: Spec(
        TX=Format("", ()),
        RX=Format("", ()),
    ),
    Type.STEP_ARM: Spec(
        TX=Format(">B", ("axis_id",)),
        RX=Format(">H", ("result",)),
    ),
    Type.STEP_DISARM: Spec(
        TX=Format(">B", ("axis_id",)),
        RX=Format(">H", ("result",)),
    ),
    Type.GET_RAW_POS: Spec(
        TX=Format(">B", ("axis_id",)),
        RX=Format(">f", ("raw_position",)),
    ),
    Type.MOVE: Spec(
        TX=Format(">BfIB", ("axis_id", "end_position", "step_frequency_hz", "flags")),
        RX=Format(">H", ("result",)),
    ),
    Type.GET_POS: Spec(
        TX=Format(">B", ("axis_id",)),
        RX=Format(">f", ("position",)),
    ),
    Type.LOG: Spec(
        TX=Format(">B", ("log_count",)),
        RX=Format(">qfffffiii", ("timestamp_us", "current", "voltage", "angle", "position", "target_position", 
                                 "step_position", "commanded_hz", "applied_hz"), is_array=True),
    ),
    Type.RESET_WATCHDOG: Spec(
        TX=Format(">B", ("axis_id",)),
        RX=Format(">H", ("result",)),
    ),
}


# Type-schema validation in case I forgot to add a schema for a frame type
missing_schemas = set(Type) - set(SCHEMAS.keys())
if missing_schemas:
    missing_list = ", ".join(f"{t.name} (0x{t.value:02X})" for t in missing_schemas)
    raise ValueError(f"Missing schema definitions for frame types: {missing_list}")


class Direction(IntEnum):
    TX = 1
    RX = 2


class Frame:
    def __init__(self, frame_type: Type, direction: Direction, **keyword_args: int | float | bytes | str | list[Any]):
        self.frame_type = frame_type
        self.direction = direction

        spec = SCHEMAS[frame_type]
        self.format = spec.TX if direction == Direction.TX else spec.RX

        if self.format.is_array:
            raw_items = keyword_args.get("items", keyword_args.get("data", []))
            self.items = self._normalize_items(raw_items)
            if self.items is None:
                raise AttributeError(f"Missing required field 'items' for frame type {frame_type.name} with array format")
            
            if not isinstance(self.items, list):
                raise TypeError(f"'items' must be a list for frame type {frame_type.name} with array format")

            # No item-wise validation here
            # I don't know how to infer data types from the format string for array items, 
            # so we will assume the user provides correct types
        
        else:
            for field in self.format.fields:
                if field in keyword_args:
                    setattr(self, field, keyword_args[field])
                else:
                    raise AttributeError(f"Missing required field '{field}' for frame type {frame_type.name}")

    def _normalize_items(self, raw_items: Any) -> list[tuple[Any, ...]] | None:
        if isinstance(raw_items, str):
            raw_items = list(raw_items.encode("utf-8"))
        elif isinstance(raw_items, bytes):
            raw_items = list(raw_items)

        if not isinstance(raw_items, list):
            raise TypeError(f"'items' must be a list, str, or bytes, got {type(raw_items).__name__}")

        normalized: list[tuple[Any, ...]] = []
        num_fields = len(self.format.fields)

        for idx, item in enumerate(raw_items):
            if isinstance(item, dict):
                # Convert dictionary item to tuple matching self.format.fields order
                tuple_item = tuple(item[f] for f in self.format.fields)
                normalized.append(tuple_item)
            elif isinstance(item, (tuple, list)):
                normalized.append(tuple(item))
            elif num_fields == 1:
                # Single-field array item (e.g. ECHO bytes or ints) -> wrap into 1-tuple automatically
                normalized.append((item,))
            else:
                raise ValueError(
                    f"Item at index {idx} must be a tuple/list or dict of {num_fields} fields"
                )

        return normalized

    def pack(self) -> bytes:
        """Serializes the frame into bytes compatible with the C implementation."""
        if self.format.is_array:
            assert self.items is not None
            data_chunks = []
            for item in self.items:
                chunk = struct.pack(self.format.byte_format, *item)
                data_chunks.append(chunk)
            data = b"".join(data_chunks)

        else:
            data_values = [getattr(self, field) for field in self.format.fields]
            data = struct.pack(self.format.byte_format, *data_values)

        if len(data) > MAX_FRAME_DATA_LENGTH:
            raise ValueError(f"Data length exceeds maximum allowed size of {MAX_FRAME_DATA_LENGTH} bytes. Actual size: {len(data)} bytes.")

        frame_type_bytes = struct.pack(">B", self.frame_type)
        data_length_bytes = struct.pack(">B", len(data))

        bytes_to_check = frame_type_bytes + data_length_bytes + data
        checksum_bytes = struct.pack(">B", sum(bytes_to_check) % 256)

        return frame_type_bytes + data_length_bytes + checksum_bytes + data


    @classmethod
    def unpack(cls, raw_bytes: bytes, direction: Direction = Direction.RX) -> "Frame":
        """Deserializes raw bytes into a Frame object, validating the checksum and data length."""

        if len(raw_bytes) < MIN_FRAME_DATA_LENGTH:
            raise ValueError(f"Raw bytes too short to be a valid frame. Minimum length is {MIN_FRAME_DATA_LENGTH} bytes.")

        frame_type_val: int = raw_bytes[0]
        data_length: int = raw_bytes[1]
        recieved_checksum: int = raw_bytes[2]

        try:
            frame_type: Type = Type(frame_type_val)
        except ValueError:
            raise ValueError(f"Unknown frame type: 0x{frame_type_val:02X}")

        expected_total_length = 3 + data_length
        if len(raw_bytes) < expected_total_length:
            raise ValueError(f"Raw bytes length {len(raw_bytes)} is less than expected length {expected_total_length} for frame type {frame_type.name}.")

        data = raw_bytes[3:expected_total_length]

        bytes_to_check = bytes([frame_type_val, data_length]) + data
        calculated_checksum = sum(bytes_to_check) % 256
        if recieved_checksum != calculated_checksum:
            raise ValueError(
                f"Checksum mismatch for frame type {frame_type.name}: "
                f"expected {recieved_checksum}, calculated {calculated_checksum}."
            )

        spec = SCHEMAS[frame_type]
        format_to_use = spec.TX if direction == Direction.TX else spec.RX

        if format_to_use.is_array:
            if not format_to_use.byte_format:
                items = []
            else:
                item_size = struct.calcsize(format_to_use.byte_format)
                if item_size == 0:
                    raise ValueError(f"Invalid byte format '{format_to_use.byte_format}' for frame type {frame_type.name}.")

                if len(data) % item_size != 0:
                    raise ValueError(f"Data length {len(data)} is not a multiple of item size {item_size} for frame type {frame_type.name}.")

                items = [list(row) for row in struct.iter_unpack(format_to_use.byte_format, data)]

            return cls(frame_type, direction, items=items)

        else:
            if not format_to_use.byte_format:
                keyword_args = {}
            else:
                expected_data_length = struct.calcsize(format_to_use.byte_format)
                if len(data) != expected_data_length:
                    raise ValueError(f"Data length {len(data)} does not match expected length {expected_data_length} for frame type {frame_type.name}.")

                unpacked_data = struct.unpack(format_to_use.byte_format, data)
                keyword_args = dict(zip(format_to_use.fields, unpacked_data))

            return cls(frame_type, direction, **keyword_args)

    def __getattr__(self, name: str) -> Any:
        if self.format.is_array and name == "items":
            return self.items
        elif not self.format.is_array and name in self.format.fields:
            return self.name
        else:
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
