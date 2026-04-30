"""NDI runtime location helpers.

Resolves where libndi.so.6 lives on disk so the bridge subprocess can dlopen
it and Python ctypes can find it for discovery. Mirrors the experiment
collector's resolver so a single SDK install serves both workspaces.

Search order:
    1. IMAGING_STATION_NDI_RUNTIME_DIR env var (set by lite-native activation)
    2. ~/.ndi-sdk/lib (where scripts/setup-ndi-host.sh writes)
    3. NDI_RUNTIME_DIR_V6 env var (NDI-SDK convention; honoured by the bridge)
    4. $CONDA_PREFIX/lib (pixi env, if libndi was installed there manually)
    5. /usr/local/lib (system install fallback)
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping


def _conda_lib_dir() -> Path:
    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if conda_prefix:
        return Path(conda_prefix) / "lib"
    return Path("/usr/local/lib")


def _owned_runtime_dir() -> Path:
    # Where scripts/setup-ndi-host.sh installs the SDK. Same path serves the
    # CMake build (FindNDISdk picks up the headers from $HOME/.ndi-sdk/include)
    # and the runtime dlopen of libndi.so.6.
    return Path.home() / ".ndi-sdk" / "lib"


def preferred_ndi_runtime_dir() -> Path:
    """Best guess at the directory containing libndi.so.6."""
    override = os.environ.get("IMAGING_STATION_NDI_RUNTIME_DIR", "").strip()
    if override:
        override_path = Path(override)
        if (override_path / "libndi.so.6").is_file():
            return override_path

    owned_dir = _owned_runtime_dir()
    if (owned_dir / "libndi.so.6").is_file():
        return owned_dir

    runtime_dir = os.environ.get("NDI_RUNTIME_DIR_V6", "").strip()
    if runtime_dir:
        runtime_path = Path(runtime_dir)
        if (runtime_path / "libndi.so.6").is_file():
            return runtime_path

    return _conda_lib_dir()


def _existing_dirs(parts: list[Path]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for part in parts:
        value = str(part)
        if not value or value in seen or not part.is_dir():
            continue
        seen.add(value)
        result.append(value)
    return result


def ndi_library_path() -> str:
    """Colon-joined LD_LIBRARY_PATH-style string covering every plausible NDI dir."""
    base_parts = [preferred_ndi_runtime_dir(), _conda_lib_dir(), Path("/usr/local/lib")]
    extra_parts = [Path(p) for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p]
    return ":".join(_existing_dirs(base_parts + extra_parts))


def build_ndi_runtime_env(extra: Mapping[str, str] | None = None) -> dict[str, str]:
    """Environment dict with NDI_RUNTIME_DIR_V6 + LD_LIBRARY_PATH set for the bridge."""
    env = dict(os.environ)
    env["NDI_RUNTIME_DIR_V6"] = str(preferred_ndi_runtime_dir())
    env["LD_LIBRARY_PATH"] = ndi_library_path()
    if extra:
        env.update(extra)
    return env
