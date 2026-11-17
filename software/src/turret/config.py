import dataclasses
import typing
from dataclasses import dataclass
from typing import Any, Type
from pathlib import Path

import yaml


class ConfigError(ValueError):
    ...


@dataclass
class SerialPortConfig:
    port: str
    

@dataclass
class SerialConfig:
    logs: SerialPortConfig
    comms: SerialPortConfig
    baudrate: int
    reply_timeout_s: float
    trigger_timeout_s: float
    poll_hz: float
    reconnect_s: float
    esp_log_file: str


@dataclass
class CameraConfig:
    index: int
    width: int
    height: int
    fps: int
    retry_s: float


@dataclass
class ControlConfig:
    pan_axis_id: int
    tilt_axis_id: int
    hfov_deg: float
    invert_pan: bool
    invert_tilt: bool
    gain: float
    deadband_deg: float
    lead_time_s: float
    move_rate_hz: float
    pan_min_deg: float
    pan_max_deg: float
    tilt_min_deg: float
    tilt_max_deg: float
    fire_tolerance_deg: float
    fire_stable_frames: int
    fire_cooldown_s: float
    position_max_age_s: float


@dataclass
class UiConfig:
    host: str
    port: int
    jpeg_quality: int
    stream_fps: float
    log_lines: int


@dataclass
class AppConfig:
    serial: SerialConfig
    camera: CameraConfig
    control: ControlConfig
    ui: UiConfig

    def __post_init__(self) -> None:
        if self.serial.comms.port == self.serial.logs.port:
            raise ConfigError("serial.comms.port cannot be serial.logs.port")
        if self.control.pan_axis_id == self.control.tilt_axis_id:
            raise ConfigError("control.pan_axis_id cannot be control.tilt_axis_id")
        if self.control.pan_min_deg > self.control.pan_max_deg:
            raise ConfigError("control.pan_min_deg cannot exceed control.pan_max_deg")
        if self.control.tilt_min_deg > self.control.tilt_max_deg:
            raise ConfigError("control.tilt_min_deg cannot exceed control.tilt_max_deg")


def _build[T](cls: Type[T], data: Any, yaml_path: str) -> T:
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ConfigError(f"{yaml_path}: expected a mapping, got {type(data).__name__}")

    hints = typing.get_type_hints(cls)
    names = {f.name for f in dataclasses.fields(cls)}  # type: ignore[arg-type]
    unknown = set(data) - names
    if unknown:
        raise ConfigError(f"{yaml_path}: unknown key(s): {", ".join(sorted(unknown))}")
    missing = names - set(data)
    if missing:
        raise ConfigError(f"{yaml_path}: missing key(s): {", ".join(sorted(missing))}")

    kwargs: dict[str, Any] = {}
    for name in names:
        hint: type = hints[name]
        if dataclasses.is_dataclass(hint):
            kwargs[name] = _build(hint, data[name], f"{yaml_path}.{name}")
        else:
            kwargs[name] = data[name]

    return cls(**kwargs)


def load_config(path: Path) -> AppConfig:
    data: Any = {}
    with path.open(mode="r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    config: AppConfig = _build(AppConfig, data, "config")

    return config

# ----------------------- Test -----------------------


if __name__ == "__main__":
    own_location: Path = Path(__file__).resolve().parent
    config_location: Path = own_location / ".." / ".." / "config" / "default.yaml"
    test_config: AppConfig = load_config(config_location)

    def print_recursive(config: dict[str, Any], deep: int) -> None:
        for field, value in config.items():
            print(f"{deep * "  " + field} ", end="")
            if isinstance(value, dict):
                print("")
                print_recursive(value, deep+1)
            else:
                print(value)

    print_recursive(dataclasses.asdict(test_config), 0)
