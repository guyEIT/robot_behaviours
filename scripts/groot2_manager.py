#!/usr/bin/env python3
"""Manage the host-native Groot2 binary from a repo config file."""

from __future__ import annotations

import argparse
import json
import shutil
import stat
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "config" / "groot2.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command",
        choices=["config", "ensure", "path", "run"],
        help="Operation to perform",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Path to the Groot2 config TOML file",
    )
    parser.add_argument(
        "--platform-key",
        help="Config platform key to use (for example: macos, linux)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Redownload even if the binary already exists",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would happen without downloading or launching",
    )
    return parser.parse_args()


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def infer_platform_key() -> str:
    if sys.platform == "darwin":
        return "macos"
    if sys.platform.startswith("linux"):
        return "linux"
    raise SystemExit(f"Unsupported host platform: {sys.platform}")


def resolve_platform_config(config: dict, platform_key: str) -> dict:
    platforms = config.get("platform", {})
    if platform_key not in platforms:
        raise SystemExit(f"No Groot2 config found for platform '{platform_key}'")

    install_dir = REPO_ROOT / config["install"]["dir"]
    platform_cfg = dict(platforms[platform_key])
    filename = platform_cfg["filename"]
    binary_path = install_dir / filename
    launch_cmd = [
        item.format(
            install_dir=install_dir.as_posix(),
            filename=filename,
            binary_path=binary_path.as_posix(),
        )
        for item in platform_cfg.get("launch", ["{binary_path}"])
    ]

    return {
        "platform_key": platform_key,
        "install_dir": install_dir,
        "binary_path": binary_path,
        "url": platform_cfg.get("url"),
        "download_page": platform_cfg.get("download_page"),
        "note": platform_cfg.get("note"),
        "launch_cmd": launch_cmd,
    }


def ensure_binary(cfg: dict, force: bool, dry_run: bool) -> Path:
    binary_path = cfg["binary_path"]
    if binary_path.is_file() and not force:
        print(f"Groot2 already present at {binary_path}")
        return binary_path

    if not cfg.get("url"):
        message = [f"No managed download is configured for {cfg['platform_key']}."]
        if cfg.get("note"):
            message.append(cfg["note"])
        if cfg.get("download_page"):
            message.append(f"Check: {cfg['download_page']}")
        message.append(
            f"If you already have a binary, place it at {binary_path} and rerun the task."
        )
        raise SystemExit("\n".join(message))

    print(f"Preparing Groot2 for {cfg['platform_key']} at {binary_path}")
    if dry_run:
        return binary_path

    cfg["install_dir"].mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=cfg["install_dir"], delete=False, prefix="groot2.", suffix=".tmp"
    ) as temp_handle:
        temp_path = Path(temp_handle.name)

    try:
        request = urllib.request.Request(
            cfg["url"],
            headers={"User-Agent": "robot-skills-groot2-manager/1.0"},
        )
        with urllib.request.urlopen(request) as response, temp_path.open("wb") as out:
            shutil.copyfileobj(response, out)

        current_mode = temp_path.stat().st_mode
        temp_path.chmod(current_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        temp_path.replace(binary_path)
    finally:
        if temp_path.exists():
            temp_path.unlink()

    print(f"Groot2 downloaded to {binary_path}")
    return binary_path


def main() -> int:
    args = parse_args()
    config_path = Path(args.config)
    config = load_config(config_path)
    platform_key = args.platform_key or infer_platform_key()
    cfg = resolve_platform_config(config, platform_key)

    if args.command == "config":
        printable = {
            "config": str(config_path),
            "platform": cfg["platform_key"],
            "install_dir": str(cfg["install_dir"]),
            "binary_path": str(cfg["binary_path"]),
            "url": cfg["url"],
            "download_page": cfg.get("download_page"),
            "note": cfg.get("note"),
            "launch_cmd": cfg["launch_cmd"],
            "exists": cfg["binary_path"].is_file(),
        }
        print(json.dumps(printable, indent=2))
        return 0

    if args.command == "path":
        print(cfg["binary_path"])
        return 0

    ensure_binary(cfg, force=args.force, dry_run=args.dry_run)

    if args.command == "ensure":
        return 0

    print("Launching Groot2:", " ".join(cfg["launch_cmd"]))
    if args.dry_run:
        return 0
    completed = subprocess.run(cfg["launch_cmd"], cwd=REPO_ROOT, check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
