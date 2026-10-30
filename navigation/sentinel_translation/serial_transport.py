#!/usr/bin/env python3
"""Bounded, latest-only POSIX serial output. This module grants no motion permission.

Constructing a transport does not open a device. Call ``open()`` only after the
application has checked its enable gate. Call ``submit(frame, ttl_s=..., stop=...)``
and ``pump()`` from the same thread; there is no worker, queue or automatic retry.
``pump`` returns True only when the pending frame has reached the kernel in full.
That does not acknowledge reception or stopping by the controller.

A stop replaces any unstarted motion frame and cannot be replaced by motion.
Replacing, expiring or interrupting a partially written frame closes the port:
continuing with another frame would corrupt the stream. The controller must have
an independent command timeout because a failed port cannot guarantee delivery
of a stop frame. ``close(stop_frame=...)`` attempts one bounded stop, then closes.
"""

from __future__ import annotations

import math
import os
import select
import stat
import struct
import sys
import time
from dataclasses import dataclass

try:
    import termios
    import fcntl
except ImportError:  # Allow safe inspection/import on a Windows development host.
    termios = None


class TransportError(RuntimeError):
    """The port has been closed and must be explicitly re-created/re-enabled."""


@dataclass
class _Pending:
    data: bytes
    expires_at: float
    stop: bool
    offset: int = 0


def _positive_seconds(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return value


class SerialTransport:
    """Single-threaded raw 8N1 transport with one latest pending frame.

    ``write_timeout_s`` starts at submission, not at a later pump. A shorter TTL
    can be supplied per frame. After any TransportError this instance stays
    faulted; it never reopens a device or replays a pending frame.
    """

    def __init__(self, port: str, baud: int = 921600,
                 write_timeout_s: float = 0.05, max_frame_bytes: int = 65536):
        if not isinstance(port, str) or not port or "\x00" in port:
            raise ValueError("port must be a nonempty device path")
        if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
            raise ValueError("baud must be a positive integer")
        if isinstance(max_frame_bytes, bool) or not isinstance(max_frame_bytes, int) or max_frame_bytes <= 0:
            raise ValueError("max_frame_bytes must be a positive integer")
        self.port = port
        self.baud = baud
        self.write_timeout_s = _positive_seconds(write_timeout_s, "write_timeout_s")
        self.max_frame_bytes = max_frame_bytes
        self._fd: int | None = None
        self._pending: _Pending | None = None
        self._ever_opened = False
        self.fault: str | None = None
        self.frames_sent = 0
        self.stop_frames_sent = 0
        self.bytes_written = 0
        self.frames_replaced = 0
        self.frames_expired = 0
        self.motion_ignored_for_stop = 0

    @property
    def is_open(self) -> bool:
        return self._fd is not None

    @property
    def pending_bytes(self) -> int:
        return 0 if self._pending is None else len(self._pending.data) - self._pending.offset

    def open(self) -> "SerialTransport":
        if self._ever_opened or self.fault is not None:
            raise TransportError("a closed or faulted transport cannot be reopened")
        if not sys.platform.startswith('linux') or termios is None:
            raise TransportError("serial output requires Linux termios and /proc ownership checks")
        speed = getattr(termios, f"B{self.baud}", None)
        if speed is None:
            raise ValueError(f"baud {self.baud} is not supported by this termios runtime")
        fd = None
        exclusive = False
        try:
            flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
            flags |= getattr(os, "O_CLOEXEC", 0)
            fd = os.open(self.port, flags)
            if not stat.S_ISCHR(os.fstat(fd).st_mode):
                raise TransportError('serial endpoint must be a character device')
            # Linux asm-generic/ioctls.h: _IOR('T',0x40,int). Python 3.10 may omit it.
            current = fcntl.ioctl(fd, getattr(termios,'TIOCGEXCL',0x80045440), struct.pack('i', 0))
            if struct.unpack('i', current)[0]:
                raise TransportError('serial endpoint already has an exclusive owner')
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            exclusive = True
            self._verify_sole_owner(fd)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0  # No input translations or software flow control.
            attrs[1] = 0  # No output transformations.
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0  # No echo, canonical mode or terminal signal processing.
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            self._fd = fd
            self._ever_opened = True
            return self
        except (OSError, termios.error, TransportError) as exc:
            if fd is not None:
                if exclusive:
                    try:
                        fcntl.ioctl(fd, termios.TIOCNXCL)
                    except OSError:
                        pass
                try:
                    os.close(fd)
                except OSError:
                    pass
            self.fault = f"serial open/configuration failed: {exc}"
            raise TransportError(self.fault) from exc

    @staticmethod
    def _verify_sole_owner(fd):
        """TIOCEXCL stops new opens; /proc detects an earlier non-cooperating owner."""
        device = os.fstat(fd).st_rdev
        for process in os.scandir('/proc'):
            if not process.name.isdecimal():
                continue
            try:
                entries = list(os.scandir(process.path+'/fd'))
                for entry in entries:
                    if int(process.name) == os.getpid() and int(entry.name) == fd:
                        continue
                    try:
                        info = os.stat(entry.path)
                    except FileNotFoundError:
                        continue
                    if stat.S_ISCHR(info.st_mode) and info.st_rdev == device:
                        raise TransportError('serial endpoint is already open in PID '+process.name)
            except FileNotFoundError:
                continue
            except PermissionError as error:
                raise TransportError('cannot inspect serial ownership in /proc; use an authorized account with access') from error

    def _require_open(self) -> None:
        if self._fd is None:
            raise TransportError(self.fault or "transport is not open")

    def _close_fd(self) -> None:
        fd, self._fd = self._fd, None
        self._pending = None
        if fd is not None:
            try:
                try:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
                finally:
                    os.close(fd)
            except OSError as exc:
                self.fault = self.fault or f"serial close failed: {exc}"

    def _fail(self, reason: str) -> None:
        self.fault = reason
        self._close_fd()
        raise TransportError(reason)

    def _expire_pending(self, now: float) -> None:
        pending = self._pending
        if pending is None or now < pending.expires_at:
            return
        if pending.offset:
            self._fail("partially written frame expired; port closed without sending another frame")
        if pending.stop:
            self._fail("stop frame expired before delivery; controller timeout required")
        self.frames_expired += 1
        self._pending = None

    def submit(self, frame: bytes, ttl_s: float | None = None, *, stop: bool = False) -> bool:
        """Replace an unstarted frame. False means a pending stop kept priority.

        A duplicate partial frame keeps its original deadline and offset. Any
        different replacement of a partial frame faults and closes the port.
        """
        self._require_open()
        if not isinstance(frame, (bytes, bytearray, memoryview)):
            raise TypeError("frame must be bytes-like")
        data = bytes(frame)
        if not data or len(data) > self.max_frame_bytes:
            raise ValueError(f"frame length must be 1..{self.max_frame_bytes} bytes")
        ttl = self.write_timeout_s if ttl_s is None else _positive_seconds(ttl_s, "ttl_s")
        now = time.monotonic()
        self._expire_pending(now)
        pending = self._pending
        if pending is not None:
            if pending.stop and not stop:
                self.motion_ignored_for_stop += 1
                return False
            if pending.offset:
                if pending.data == data and pending.stop == bool(stop):
                    return True
                self._fail("partially written frame interrupted by replacement; port closed")
            self.frames_replaced += 1
        self._pending = _Pending(data, now + min(ttl, self.write_timeout_s), bool(stop))
        return True

    def pump(self, timeout_s: float = 0.0) -> bool:
        """Attempt a bounded write; zero timeout performs at most one write call."""
        self._require_open()
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("timeout_s must be finite and nonnegative")
        wait_until = time.monotonic() + timeout_s
        try:
            while self._pending is not None:
                now = time.monotonic()
                self._expire_pending(now)
                pending = self._pending
                if pending is None:
                    return False
                if timeout_s:
                    remaining = min(wait_until, pending.expires_at) - now
                    if remaining <= 0:
                        return False
                    _, writable, _ = select.select([], [self._fd], [], remaining)
                    self._expire_pending(time.monotonic())
                    if not writable:
                        return False
                    pending = self._pending
                    if pending is None:
                        return False
                try:
                    count = os.write(self._fd, memoryview(pending.data)[pending.offset:])
                except BlockingIOError:
                    return False
                if count <= 0:
                    self._fail("serial write made no progress; port closed")
                pending.offset += count
                self.bytes_written += count
                if pending.offset == len(pending.data):
                    self.frames_sent += 1
                    self.stop_frames_sent += int(pending.stop)
                    self._pending = None
                    return True
                if not timeout_s:
                    return False
        except (OSError, ValueError) as exc:
            self._fail(f"serial write/select failed: {exc}")
        except BaseException:
            # An interrupted partial write may never be resumed by an outer loop.
            if self._pending is not None and self._pending.offset:
                self.fault = "partially written frame interrupted; port closed"
                self._close_fd()
            raise
        return False

    def read_available(self, max_bytes: int = 4096) -> bytes:
        """Read currently available bytes once; the caller validates framing/CRC.

        Empty bytes means that no input is ready. Ready EOF, hangup and OS errors
        fault and close the transport. Receiving bytes alone proves no health.
        """
        self._require_open()
        if isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or not 1 <= max_bytes <= 1048576:
            raise ValueError("max_bytes must be an integer in 1..1048576")
        try:
            readable, _, _ = select.select([self._fd], [], [], 0)
            if not readable:
                return b""
            data = os.read(self._fd, max_bytes)
            if not data:
                self._fail("serial input EOF/hangup; port closed")
            return data
        except BlockingIOError:
            return b""
        except (OSError, ValueError) as exc:
            self._fail(f"serial read/select failed: {exc}")

    def close(self, stop_frame: bytes | None = None, timeout_s: float | None = None) -> bool:
        """Return True only if an optional stop frame was fully written.

        Write failures are raised after closing. No tcdrain is used: it can block
        indefinitely, and kernel acceptance is not a controller acknowledgement.
        A close without stop_frame is descriptor cleanup only and returns False.
        """
        if stop_frame is None:
            self._close_fd()
            return False
        if self._fd is None:
            raise TransportError(self.fault or "cannot send stop: transport is closed")
        budget = self.write_timeout_s if timeout_s is None else _positive_seconds(timeout_s, "timeout_s")
        budget = min(budget, self.write_timeout_s)
        try:
            self.submit(stop_frame, ttl_s=budget, stop=True)
            complete = self.pump(timeout_s=budget)
            if not complete:
                self._fail("stop frame was not fully written within deadline; controller timeout required")
            return True
        finally:
            self._close_fd()

    def status(self) -> dict:
        return {"open": self.is_open, "fault": self.fault,
                "pending_bytes": self.pending_bytes, "frames_sent": self.frames_sent,
                "stop_frames_sent": self.stop_frames_sent, "bytes_written": self.bytes_written,
                "frames_replaced": self.frames_replaced, "frames_expired": self.frames_expired,
                "motion_ignored_for_stop": self.motion_ignored_for_stop}
