import json
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any
from urllib.parse import urlencode

import typer
import yaml

app = typer.Typer(add_completion=False, no_args_is_help=True,
                  help="Ball-tracking turret for Raspberry Pi 5 + ESP32-S3.")
URL = typer.Option("http://127.0.0.1:8000", "--url", envvar="TURRET_URL", help="Address of the running turret.")


def call(url: str, path: str, method: str = "GET") -> Any:
    request = urllib.request.Request(url.rstrip("/") + path, method=method,
                                     data=b"" if method == "POST" else None)
    try:
        with urllib.request.urlopen(request, timeout=3) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        typer.secho(f"request to {url} failed: HTTP {exc.code} {exc.reason}",
                    fg=typer.colors.RED, err=True)
        raise typer.Exit(1) from exc
    except (urllib.error.URLError, OSError, ValueError) as exc:
        typer.secho(f"cannot reach {url}: {exc}", fg=typer.colors.RED, err=True)
        raise typer.Exit(1) from exc


def show_status(status: dict) -> None:
    for source, fields in status.items():
        parts = []
        for key, value in fields.items():
            if key in ("path", "bbox"):
                continue
            parts.append(f"{key}={value:.1f}" if isinstance(value, float) else f"{key}={value}")
        typer.echo(f"{source:<10} " + "  ".join(parts))


@app.command()
def run(config: Path = typer.Option(Path("config/default.yaml"), "--config", "-c",
                                    exists=True, dir_okay=False, help="YAML config file.")) -> None:
    """Start all workers. Ctrl+C stops them."""
    from .app import run as run_app
    from .config import ConfigError, load_config

    try:
        cfg = load_config(config)
    except (ConfigError, yaml.YAMLError) as exc:
        typer.secho(f"config error: {exc}", fg=typer.colors.RED, err=True)
        raise typer.Exit(2) from exc
    run_app(cfg)


@app.command()
def grid(
    url: str = URL,
    spacing: int = typer.Option(20, min=1, max=10000, help="Grid spacing in pixels."),
    width: int = typer.Option(2, min=1, max=20, help="Grid line width in pixels."),
    off: bool = typer.Option(False, "--off", help="Hide the calibration grid."),
) -> None:
    """Show or hide a calibration grid on the UI."""
    params = urlencode({"enabled": str(not off).lower(), "spacing": spacing, "width": width})
    call(url, f"/api/grid?{params}", "POST")
    if off:
        typer.echo("calibration grid disabled")
    else:
        typer.echo(f"calibration grid enabled (spacing={spacing}, width={width})")


@app.command()
def calibrate(
    url: str = URL,
    distance_m: float = 1.0
) -> None:
    """Calibrate distance (in meters) between the turret and the target"""
    param = urlencode({"distance": distance_m})
    call(url, f"/api/calibrate?{param}", "POST")
    typer.secho(f"calibrated distance set to {distance_m} meters")


@app.command()
def point(
    x: int = typer.Argument(..., min=0, help="Horizontal image coordinate in pixels."),
    y: int = typer.Argument(..., min=0, help="Vertical image coordinate in pixels."),
    url: str = URL,
) -> None:
    """Send an image-space point selection to the UI."""
    params = urlencode({"x": x, "y": y})
    result = call(url, f"/api/point?{params}", "POST")
    typer.echo(f"point sent: x={result['x']}, y={result['y']}")


@app.command()
def move(
    pan: float = typer.Argument(..., help="Pan target angle in degrees."),
    tilt: float = typer.Argument(..., help="Tilt target angle in degrees."),
    config: Path = typer.Option(Path("config/default.yaml"), "--config", "-c",
                                exists=True, dir_okay=False, help="YAML config file."),
    url: str = URL,
) -> None:
    """Move the pan and tilt axes to absolute angles within their configured limits."""
    from .config import ConfigError, load_config

    try:
        cfg = load_config(config)
    except (ConfigError, yaml.YAMLError) as exc:
        typer.secho(f"config error: {exc}", fg=typer.colors.RED, err=True)
        raise typer.Exit(2) from exc

    c = cfg.control
    if not c.pan_min_deg <= pan <= c.pan_max_deg:
        typer.secho(f"pan must be between {c.pan_min_deg} and {c.pan_max_deg} degrees",
                    fg=typer.colors.RED, err=True)
        raise typer.Exit(2)
    if not c.tilt_min_deg <= tilt <= c.tilt_max_deg:
        typer.secho(f"tilt must be between {c.tilt_min_deg} and {c.tilt_max_deg} degrees",
                    fg=typer.colors.RED, err=True)
        raise typer.Exit(2)

    params = urlencode({"pan": pan, "tilt": tilt})
    call(url, f"/api/move?{params}", "POST")
    typer.echo(f"move sent: pan={pan}, tilt={tilt}")


@app.command()
def status(url: str = URL) -> None:
    """Print the current status once."""
    show_status(call(url, "/api/status"))


@app.command()
def watch(url: str = URL, interval: float = 0.5, lines: int = 8) -> None:
    """Live status plus the latest ESP debug lines. Ctrl+C to leave."""
    try:
        while True:
            state = call(url, "/api/status")
            logs = call(url, f"/api/logs?n={lines}")
            typer.echo("\x1b[2J\x1b[H", nl=False)        # clear the screen
            show_status(state)
            typer.echo("\nESP debug (UART0):")
            for entry in logs:
                typer.echo(f"  {time.strftime('%H:%M:%S', time.localtime(entry['ts']))} {entry['line']}")
            time.sleep(interval)
    except KeyboardInterrupt:
        pass


@app.command()
def arm(url: str = URL, yes: bool = typer.Option(False, "--yes", "-y")) -> None:
    """ARM: the turret will aim and fire at a tracked ball."""
    if not yes:
        typer.confirm("Arm the turret? It will move and fire automatically.", abort=True)
    call(url, "/api/arm", "POST")
    typer.secho("armed", fg=typer.colors.YELLOW)


@app.command()
def disarm(url: str = URL) -> None:
    """DISARM: vision keeps running, no MOVE or TRIGGER is sent."""
    call(url, "/api/disarm", "POST")
    typer.secho("disarmed", fg=typer.colors.GREEN)


@app.command()
def reset_error(url: str = URL, yes: bool = typer.Option(False, "--yes", "-y")) -> None:
    """Send RESET_ERROR after an axis error. Reset the axis by hand first; then arm again."""
    if not yes:
        typer.confirm("Has the axis been reset by hand?", abort=True)
    call(url, "/api/reset_error", "POST")
    typer.secho("RESET_ERROR sent; run `turret arm` to continue", fg=typer.colors.YELLOW)
