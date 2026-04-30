"""System-mDNS NDI discovery via avahi-browse.

Used as a fallback when libndi's built-in mDNS implementation can't see the
source — common on multi-interface hosts (Linux multicast picks one route,
which may not be the one the source advertises on). avahi-daemon doesn't
suffer this because it binds to every interface.

Shells out to `avahi-browse -p -t -r _ndi._tcp`. Output rows look like:

    =;eno1;IPv4;ZOWIEBOX-40435\\032\\040ZowieBox-40435\\041;_ndi._tcp;local;\\
        ZowieBox-40435.local;10.6.105.98;5961;...

(The `\\NNN` escapes are decimal byte values, not C-octal.)
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

_ESCAPE_RE = re.compile(r"\\(\d{3})")

# Pixi-managed envs typically don't inherit DBUS_SYSTEM_BUS_ADDRESS, which
# avahi-browse needs to talk to the host's avahi-daemon. Default to the
# standard Linux system-bus socket if the env doesn't already set it.
_SYSTEM_BUS_CANDIDATES = (
    "/run/dbus/system_bus_socket",
    "/var/run/dbus/system_bus_socket",
)


def _build_env() -> dict:
    env = dict(os.environ)
    if "DBUS_SYSTEM_BUS_ADDRESS" not in env:
        for sock in _SYSTEM_BUS_CANDIDATES:
            if Path(sock).exists():
                env["DBUS_SYSTEM_BUS_ADDRESS"] = f"unix:path={sock}"
                break
    return env


def _unescape(value: str) -> str:
    return _ESCAPE_RE.sub(lambda m: chr(int(m.group(1))), value)


@dataclass(slots=True)
class AvahiSource:
    name: str          # e.g. "ZOWIEBOX-40435 (ZowieBox-40435)"
    address: str       # e.g. "10.6.105.98"
    port: int          # e.g. 5961
    interface: str     # e.g. "eno1"

    @property
    def url_address(self) -> str:
        """host:port form expected by NDIlib_recv_create_v3.source_to_connect_to.p_url_address."""
        return f"{self.address}:{self.port}"


def avahi_available() -> bool:
    return shutil.which("avahi-browse") is not None


def discover(timeout_sec: float = 4.0) -> list[AvahiSource]:
    """Return resolved NDI sources visible to avahi-daemon, deduplicated by name."""
    if not avahi_available():
        return []
    try:
        result = subprocess.run(
            ["avahi-browse", "-p", "-t", "-r", "_ndi._tcp"],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
            env=_build_env(),
        )
    except subprocess.TimeoutExpired:
        return []
    sources: dict[str, AvahiSource] = {}
    for line in result.stdout.splitlines():
        # Resolved rows start with '=' and have 10 fields:
        # =;<iface>;<proto>;<name>;<svctype>;<domain>;<host>;<address>;<port>;<txt>
        parts = line.split(";")
        if not parts or parts[0] != "=" or len(parts) < 9:
            continue
        if parts[2] != "IPv4":
            continue
        name = _unescape(parts[3])
        try:
            port = int(parts[8])
        except ValueError:
            continue
        sources.setdefault(
            name,
            AvahiSource(name=name, address=parts[7], port=port, interface=parts[1]),
        )
    return list(sources.values())


def find_source(name: str, timeout_sec: float = 4.0) -> AvahiSource | None:
    for src in discover(timeout_sec=timeout_sec):
        if src.name == name:
            return src
    return None
