"""Drive the actual C++ Session against the Python Board over process pipes.

Virtual monotonic time, no serial/network/hardware. The child must be the
repository's session_peer executable, which requires --stdio-offline.
"""
import argparse
import json
from pathlib import Path
import queue
import subprocess
import threading

from .protocol import StreamDecoder, encode
from .simulator import Board


class Harness:
    def __init__(self, binary):
        self.board = Board()
        self.now = 1_000_000
        self.incoming = bytearray()
        self.drop_ack = False
        self.silent = False
        self.frames_sent = 0
        self.process = subprocess.Popen([str(Path(binary).resolve()), "--stdio-offline"],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, bufsize=1)
        self.lines = queue.Queue()

        def reader():
            for line in self.process.stdout:
                self.lines.put(line)
            self.lines.put(None)
        self.reader = threading.Thread(target=reader, daemon=True)
        self.reader.start()
        self.last = self.step(delta=0)

    def close(self):
        try:
            self.process.stdin.close()
            self.process.wait(timeout=3)
        except (subprocess.TimeoutExpired, BrokenPipeError):
            self.process.kill()
            self.process.wait(timeout=3)
        self.reader.join(timeout=2)
        self.process.stdout.close()
        self.process.stderr.close()

    def step(self, action="none", delta=1000, fragment=None):
        self.board.advance(delta)
        self.now += delta
        count = len(self.incoming) if fragment is None else fragment
        rx = bytes(self.incoming[:count])
        del self.incoming[:count]
        self.process.stdin.write(f"{self.now} {action} {rx.hex() or '-'}\n")
        self.process.stdin.flush()
        try:
            line = self.lines.get(timeout=5)
        except queue.Empty as exc:
            raise AssertionError("C++ peer did not answer within 5 seconds") from exc
        if line is None:
            raise AssertionError(f"C++ peer exited: {self.process.stderr.read()}")
        result = json.loads(line)
        for wire in result["wires"]:
            raw = self.board.receive(bytes.fromhex(wire))
            self.frames_sent += 1
            if self.drop_ack:
                raw = b"".join(encode(f) for f in StreamDecoder().feed(raw)
                               if f.msg_type != 0x00FF)
            if not self.silent:
                self.incoming.extend(raw)
        if not self.silent:
            self.incoming.extend(self.board.telemetry())
        self.last = result
        return result

    def until(self, predicate, steps=800):
        for _ in range(steps):
            if predicate(self.last):
                return self.last
            self.step()
        raise AssertionError(f"condition not reached; last C++ state: {self.last}")

    def synchronized(self):
        self.until(lambda s: s["active"])
        # At least eight sync exchanges and a following pose/status publication.
        for _ in range(50):
            self.step()
        self.until(lambda s: not s["pending"])


def check(binary):
    results = []
    h = Harness(binary)
    try:
        h.synchronized()
        assert h.board.mode == 0 and not h.last["ready"]
        # Noise and a bad-CRC frame followed by a valid fragmented frame.
        good = h.board.telemetry(force=True)
        corrupt = bytearray(good[:62])
        if corrupt:
            corrupt[-1] ^= 0x80
        h.incoming[:0] = b"noise" + corrupt + good
        h.step(fragment=7)
        h.step()
        assert h.last["active"]
        assert h.step("mode")["action_accepted"]
        h.until(lambda s: s["ready"])
        assert h.step("aim")["action_accepted"]
        h.step(delta=20_000)
        assert h.board.last_applied != 0
        assert h.board.target is not None
        # More than 3 s exercises continuous clock resynchronization, not just startup.
        for _ in range(3600):
            h.step()
        assert h.last["active"] and h.last["ready"]
        assert h.board.target is None  # heartbeat must not extend the old target
        assert h.step("stop")["action_accepted"]
        h.until(lambda s: not s["pending"])
        assert h.board.mode == 0 and not h.last["ready"]
        results.append("handshake/sync/fragmentation/CRC/mode/aim/applied/expiry/stop")
    finally:
        h.close()

    h = Harness(binary)
    try:
        h.synchronized()
        h.drop_ack = True
        assert h.step("mode")["action_accepted"]
        for _ in range(110):
            h.step()
        assert h.last["completion"] == "timeout" and not h.last["ready"]
        h.drop_ack = False
        assert h.step("retry")["action_accepted"]
        h.until(lambda s: s["ready"])
        assert h.board.mode == 3
        # MCU reboot rejects the old session; never resumes motion automatically.
        old_session = h.last["session"]
        h.board.reboot()
        h.incoming.clear()
        h.until(lambda s: not s["active"])
        assert not h.last["ready"]
        assert h.step("connect")["action_accepted"]
        h.until(lambda s: s["active"])
        assert h.last["session"] != old_session and not h.last["ready"]
        results.append("lost_ACK/original_retry/reboot/new_session/no_auto_takeover")
    finally:
        h.close()

    h = Harness(binary)
    try:
        h.synchronized()
        h.step("mode")
        h.until(lambda s: s["ready"])
        h.step("aim")
        h.silent = True
        h.incoming.clear()
        h.until(lambda s: not s["active"])
        assert not h.last["ready"] and not h.step("aim")["action_accepted"]
        # Sender stopped: virtual MCU independently lets the heartbeat lease expire.
        h.board.advance(600_000)
        assert h.board.session == 0 and h.board.target is None
        results.append("receive_loss/client_timeout/board_lease/no_motion_replay")
    finally:
        h.close()
    return {"environment": "offline-cross-language-pipes", "hardware": False,
            "scenarios": results, "passed": len(results)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("peer", type=Path)
    args = parser.parse_args()
    print(json.dumps(check(args.peer), indent=2))


if __name__ == "__main__":
    main()
