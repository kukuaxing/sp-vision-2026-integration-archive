"""Monotonic clock mapping and bounded, non-extrapolating quaternion history."""
from collections import deque
from dataclasses import dataclass
import math


class ClockSync:
    def __init__(self):
        self.samples = deque(maxlen=8)

    def reset(self):
        self.samples.clear()

    def observe(self, t1, t2, t3, t4):
        if t4 < t1 or t3 < t2:
            raise ValueError("non-monotonic sync exchange")
        rtt = (t4 - t1) - (t3 - t2)
        if rtt < 0:
            raise ValueError("negative transport round trip")
        self.samples.append((t4, ((t2 - t1) + (t3 - t4)) / 2, rtt / 2))

    def estimate(self, host_now):
        recent = [s for s in self.samples if 0 <= host_now - s[0] <= 3_000_000]
        if len(recent) < 8:
            raise ValueError("eight recent time exchanges required")
        best = min(recent, key=lambda sample: sample[2])
        # RTT is only a bound under the assumed clock model, not exposure accuracy.
        uncertainty = max(best[2], max(abs(s[1] - best[1]) for s in recent))
        if uncertainty > 5_000:
            raise ValueError("clock uncertainty exceeds 5 ms")
        return best[1], uncertainty

    def to_mcu(self, host_time, host_now):
        offset, _ = self.estimate(host_now)
        return round(host_time + offset)


def normalized(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError("invalid quaternion")
    norm = math.sqrt(sum(v * v for v in q))
    if not 0.95 <= norm <= 1.05:
        raise ValueError("quaternion norm outside tolerance")
    return tuple(v / norm for v in q)


def slerp(a, b, fraction):
    a, b = normalized(a), normalized(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-v for v in b), -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalized(tuple(x + fraction * (y - x) for x, y in zip(a, b)))
    theta = math.acos(dot)
    left = math.sin((1 - fraction) * theta) / math.sin(theta)
    right = math.sin(fraction * theta) / math.sin(theta)
    return tuple(left * x + right * y for x, y in zip(a, b))


@dataclass(frozen=True)
class Pose:
    time_us: int
    counter: int
    epoch: int
    q: tuple


class PoseHistory:
    def __init__(self, capacity=256):
        self.samples = deque(maxlen=capacity)
        self.epoch = None

    def reset(self):
        self.samples.clear()
        self.epoch = None

    def add(self, pose, now_mcu_us):
        q = normalized(pose.q)
        if not -5_000 <= now_mcu_us - pose.time_us <= 100_000:
            raise ValueError("stale or future pose")
        if self.epoch != pose.epoch:
            if self.samples and pose.time_us <= self.samples[-1].time_us:
                raise ValueError("old epoch cannot replace newer history")
            self.reset()
            self.epoch = pose.epoch
        if self.samples:
            last = self.samples[-1]
            newer = 0 < ((pose.counter - last.counter) & 0xFFFFFFFF) < 0x80000000
            if pose.time_us <= last.time_us or not newer:
                raise ValueError("old/duplicate pose does not refresh freshness")
        self.samples.append(Pose(pose.time_us, pose.counter, pose.epoch, q))

    def at(self, sample_time_us, now_mcu_us):
        if not self.samples or not 0 <= now_mcu_us - self.samples[-1].time_us <= 100_000:
            raise ValueError("no fresh pose")
        if not self.samples[0].time_us <= sample_time_us <= self.samples[-1].time_us:
            raise ValueError("pose extrapolation forbidden")
        for pose in self.samples:
            if pose.time_us == sample_time_us:
                return pose.q
        for left, right in zip(self.samples, list(self.samples)[1:]):
            if left.time_us < sample_time_us < right.time_us:
                if right.time_us - left.time_us > 50_000:
                    raise ValueError("pose interpolation gap exceeds 50 ms")
                fraction = (sample_time_us - left.time_us) / (right.time_us - left.time_us)
                return slerp(left.q, right.q, fraction)
        raise ValueError("no bracketing pose")
