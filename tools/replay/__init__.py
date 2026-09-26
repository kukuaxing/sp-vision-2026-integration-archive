"""Portable recording and deterministic, read-only event replay.

This package does not open cameras, serial ports, networks, or control devices.
"""

from .recording import Reader, RecordingError, Writer, inspect, replay_digest, stamp

__all__ = ["Reader", "RecordingError", "Writer", "inspect", "replay_digest", "stamp"]
