"""Minimal systemd sd_notify (READY / WATCHDOG / STOPPING) without extra packages."""
from __future__ import annotations

import os
import socket


def sd_notify(message: str) -> bool:
    addr = os.environ.get("NOTIFY_SOCKET")
    if not addr:
        return False  # not started by systemd (laptop, manual run)
    if addr.startswith("@"):
        addr = "\0" + addr[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
            s.connect(addr)
            s.sendall(message.encode())
        return True
    except OSError:
        return False


def time_synced() -> bool:
    """True when systemd-timesyncd reports the clock as NTP-synchronised."""
    import subprocess
    try:
        out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() == "yes"
    except (OSError, subprocess.SubprocessError):
        return False
