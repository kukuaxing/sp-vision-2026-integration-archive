"""Logging and durable sequence helpers for a staged one-shot field candidate."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import uuid

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DECODER = HERE/'attitude_decoder.py'
DECODER_SHA = '8478047abd6630605ea668a372f47257b97cfb9ce00c5c943d45138a8007c2a8'


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        if os.name == 'posix':
            fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
    finally:
        temporary.unlink(missing_ok=True)


def persist_sequence(path, next_sequence, boot):
    if (type(next_sequence) is not int or not 0 <= next_sequence <= 65535
            or type(boot) is not int or not 0 < boot <= 0xffffffff):
        raise ValueError('Invalid sequence/boot')
    path = Path(path)
    if path.exists():
        old = json.loads(path.read_text(encoding='utf-8'))
        previous = old.get('next_sequence')
        if type(previous) is not int or not 0 <= previous <= 65535:
            raise RuntimeError('Malformed production sequence journal')
        if old.get('boot_id', boot) != boot:
            raise RuntimeError('Sequence boot differs; explicit deployment review required')
        # Caller increments exactly once before every submit; no retry/replay.
        if next_sequence != (previous+1) & 0xffff:
            raise RuntimeError('Unexpected sequence increment; concurrent writer or stale context')
    atomic_json(path, dict(next_sequence=next_sequence, boot_id=boot))


def decoder_class():
    if hashlib.sha256(DECODER.read_bytes()).hexdigest() != DECODER_SHA:
        raise RuntimeError('Chassis decoder differs from reviewed source')
    spec = importlib.util.spec_from_file_location('_field_attitude', DECODER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ChassisAttitudeDecoder


class ReceiveFanout:
    """Accept already-read immutable bytes; never reads/opens/writes a device."""
    def __init__(self, status_stream):
        self.status = status_stream
        self.attitude = decoder_class()()
        self.chunks = []
        self.error = None

    def feed(self, data, now):
        if type(data) is not bytes:
            raise TypeError('One immutable byte string from the existing owner is required')
        if data:
            self.chunks.append(dict(host_receive_monotonic_s=now, data_hex=data.hex()))
        # Observer failures must not stop the existing status decoder from seeing bytes.
        try:
            self.attitude.feed(data, now)
        except Exception as error:
            self.error = self.error or str(error)
        self.status.feed(data, now)

    def result(self, motion_begin=None):
        return dict(schema='chassis_observation_v1', decoder_sha256=DECODER_SHA,
                    error=self.error, raw_receive_chunks=self.chunks,
                    records=self.attitude.records,
                    motion_begin_monotonic_s=motion_begin,
                    role='independent observer only; original odometry guards remain authoritative',
                    measured_chassis_gyro_available=False,
                    axes_physically_verified=False, imu_time_is_sensor_sample_time=False)


def verify_bundle():
    manifest = json.loads((HERE/'bundle_manifest.json').read_text(encoding='utf-8'))
    if manifest.get('motion_authorized') is not False:
        raise RuntimeError('Candidate manifest cannot authorize physical motion')
    for name, expected in manifest['files'].items():
        path = (ROOT/name).resolve()
        if not path.is_relative_to(ROOT.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Candidate source hash differs: '+name)
    return manifest
