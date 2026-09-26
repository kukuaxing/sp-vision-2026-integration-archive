"""Integrity, clock, feedback and immutable-artifact acceptance tests; all synthetic."""

import hashlib
import io
import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

from tools.replay import Reader, RecordingError, Writer, inspect, replay_digest, stamp
from tools.replay.__main__ import main
from tools.replay.demo import FRAMES, make_demo
from tools.replay.recording import HOST, canonical, relative_file


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="sp_vision_replay_")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "capture" / "events.jsonl"

    def demo(self):
        return make_demo(self.path)

    def writer(self):
        return Writer(self.path, run_id="unit-test", source="synthetic",
                      runtime={"application": "unittest", "revision": "fixture-1", "build": "stdlib", "platform": "test"},
                      clocks={HOST: "host-1", "mcu_monotonic": "boot-1"}, frames=FRAMES,
                      configs={"test-config": "key: 123\n"})

    def mutate(self, transform):
        records = [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines()]
        transform(records)
        # Deliberately re-seal edited fixtures to exercise semantic validation separately
        # from integrity validation. Raw tamper tests below do not use this helper.
        if records[-1].get("kind") == "end":
            records[-1]["recording_sha256"] = hashlib.sha256(
                b"".join(canonical(record) + b"\n" for record in records[:-1])).hexdigest()
        self.path.write_bytes(b"".join(canonical(record) + b"\n" for record in records))

    def rejected(self, fragment=None):
        with self.assertRaises(RecordingError) as caught:
            list(Reader(self.path))
        if fragment:
            self.assertIn(fragment, str(caught.exception))

    def pose(self, writer, host=100, source=100, epoch="boot-1", mapped=None, sensor="small_y"):
        return writer.append("pose", stream=f"pose/{sensor}", host_us=host,
                             timestamp=stamp(source, domain="mcu_monotonic", epoch=epoch,
                                             mapped_host_us=mapped, uncertainty_us=0 if mapped is not None else None),
                             data={"sensor_id": sensor, "from_frame": "gimbal", "to_frame": "world",
                                   "quaternion_wxyz": [1, 0, 0, 0], "angular_velocity_rad_s": None})

    def epoch(self, writer, host=101, previous="boot-1", current="boot-2"):
        writer.append("clock_epoch", stream="clock/mcu", host_us=host, timestamp=stamp(host),
                      data={"domain": "mcu_monotonic", "previous_epoch": previous,
                            "epoch": current, "reason": "MCU boot_id changed"})

    def image(self, writer, host=100, measured=100):
        digest = writer.write_image("images/one.pgm", b"P5\n2 2\n255\n\0\1\2\3")
        writer.append("image", stream="camera/main", host_us=host, timestamp=stamp(measured),
                      data={"image_id": "one", "path": "images/one.pgm", "sha256": digest,
                            "width": 2, "height": 2, "encoding": "pgm"})

    def test_demo_full_integrity_and_no_inferred_application(self):
        report = inspect(self.demo())
        self.assertEqual(report["source"], "synthetic")
        self.assertEqual(report["event_count"], 73)
        self.assertEqual(report["event_counts"]["image"], 12)
        self.assertEqual(report["commands"]["recorded"], 12)
        self.assertEqual(report["commands"]["accepted_by_feedback"], 12)
        self.assertEqual(report["commands"]["applied_by_feedback"], 11)
        self.assertEqual(report["commands"]["without_applied_feedback"], 1)
        brackets = report["image_pose_brackets"]["by_sensor_frames_and_epoch"]
        self.assertEqual(brackets[0]["images_with_pose_bracket_at_most_50ms"], 11)
        self.assertNotIn("hit_rate", report)

    def test_replay_repeatability_is_independent_of_directory(self):
        self.demo()
        other = make_demo(self.root / "other" / "events.jsonl")
        first = replay_digest(Reader(self.path))
        self.assertEqual(first, replay_digest(Reader(self.path)))
        self.assertEqual(first, replay_digest(Reader(other)))
        self.assertEqual(first["event_sha256"], inspect(self.path)["event_sha256"])

    def test_existing_capture_and_image_are_not_overwritten(self):
        self.demo()
        before = self.path.read_bytes()
        with self.assertRaises(FileExistsError):
            self.demo()
        self.assertEqual(before, self.path.read_bytes())
        with self.writer_at_other_path() as writer:
            writer.write_image("sample.pgm", b"first")
            with self.assertRaises(FileExistsError):
                writer.write_image("sample.pgm", b"second")
            self.assertEqual((writer.path.parent / "sample.pgm").read_bytes(), b"first")

    def writer_at_other_path(self):
        self.path = self.root / "separate" / "events.jsonl"
        return self.writer()

    def test_missing_image_rejected(self):
        self.demo()
        (self.path.parent / "images/image-0000.pgm").unlink()
        self.rejected("missing image")

    def test_image_tamper_rejected(self):
        self.demo()
        image = self.path.parent / "images/image-0000.pgm"
        raw = image.read_bytes()
        image.write_bytes(raw[:-1] + b"\xff")
        self.rejected("hash mismatch")

    def test_wrong_pnm_dimensions_rejected_even_with_valid_hash(self):
        self.demo()
        self.mutate(lambda rows: rows[2]["data"].update(width=31))
        self.rejected("PNM header")

    def test_config_tamper_rejected(self):
        self.demo()
        self.mutate(lambda rows: rows[0]["configs"][0].update(content_utf8="changed: true\n"))
        self.rejected("snapshot hash mismatch")

    def test_unsafe_paths_portable(self):
        for value in ("../escape", "/absolute", "C:/windows", "C:relative", "images\\file",
                      "images/../file", "./file", "images//file", "images/file:stream", "",
                      "images/NUL", "images/con.pgm", "COM1", "LPT¹.txt", "images/name.",
                      "images/name ", "images/*.png", 'images/a"b.png'):
            with self.subTest(path=value), self.assertRaises(RecordingError):
                relative_file(self.root, value)

    def test_symlink_escape_rejected_when_platform_supports_creation(self):
        outside = self.root / "outside"
        inside = self.root / "inside"
        outside.mkdir()
        inside.mkdir()
        try:
            (inside / "link").symlink_to(outside, target_is_directory=True)
        except OSError:
            self.skipTest("OS account cannot create symlinks")
        with self.assertRaises(RecordingError):
            relative_file(inside, "link/image.pgm")

    def test_escaping_image_reference_in_json_rejected(self):
        self.demo()
        self.mutate(lambda rows: rows[2]["data"].update(path="../outside.pgm"))
        self.rejected("unsafe relative path")

    def test_truncated_last_line_rejected(self):
        self.demo()
        self.path.write_bytes(self.path.read_bytes()[:-1])
        self.rejected("unterminated")

    def test_missing_whole_footer_rejected(self):
        self.demo()
        self.path.write_bytes(b"\n".join(self.path.read_bytes().splitlines()[:-1]) + b"\n")
        self.rejected("missing end footer")

    def test_semantically_valid_event_tamper_fails_whole_recording_hash(self):
        self.demo()
        self.path.write_bytes(self.path.read_bytes().replace(b'"mode":"calibration"', b'"mode":"manual"', 1))
        self.rejected("recording hash mismatch")

    def test_external_writer_json_formatting_uses_raw_byte_hash(self):
        self.demo()
        rows = [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines()]
        # Other languages need not duplicate Python's sorted compact JSON formatting.
        encoded = b"".join((json.dumps(row, separators=(", ", ": ")) + "\r\n").encode("utf-8")
                            for row in rows[:-1])
        rows[-1]["recording_sha256"] = hashlib.sha256(encoded).hexdigest()
        self.path.write_bytes(encoded + canonical(rows[-1]) + b"\n")
        self.assertEqual(len(list(Reader(self.path))), 73)

    def test_trailing_events_after_footer_rejected(self):
        self.demo()
        original = self.path.read_bytes()
        self.path.write_bytes(original + original.splitlines()[1] + b"\n")
        self.rejected("after end footer")

    def test_aborted_writer_does_not_claim_complete_capture(self):
        with self.assertRaises(RuntimeError):
            with self.writer() as writer:
                self.pose(writer)
                raise RuntimeError("upstream capture stopped unexpectedly")
        self.rejected("marked recording incomplete")

    def test_duplicate_json_keys_rejected(self):
        self.demo()
        raw = self.path.read_bytes().replace(b'"seq":1,', b'"seq":1,"seq":1,', 1)
        self.path.write_bytes(raw)
        self.rejected("duplicate JSON key")

    def test_nonfinite_and_invalid_json_rejected(self):
        self.demo()
        self.path.write_bytes(self.path.read_bytes().replace(b'"seq":1,', b'"seq":NaN,', 1))
        self.rejected("non-finite")

    def test_missing_unknown_and_sequence_gap_rejected(self):
        self.demo()
        for mode in ("missing", "unknown", "sequence"):
            original = self.path.read_bytes()
            if mode == "missing":
                self.mutate(lambda rows: rows[1].pop("host_us"))
            elif mode == "unknown":
                self.mutate(lambda rows: rows[1].update(unrecognized=True))
            else:
                self.mutate(lambda rows: rows[1].update(seq=8))
            with self.subTest(mode=mode):
                self.rejected()
            self.path.write_bytes(original)

    def test_reader_rejects_oversized_line(self):
        self.path.parent.mkdir()
        self.path.write_bytes(b" " * 128 + b"\n")
        with patch("tools.replay.recording.MAX_LINE_BYTES", 64):
            self.rejected("exceeds 64 bytes")

    def test_empty_recording_rejected_manifest_only_allowed(self):
        self.path.parent.mkdir()
        self.path.write_bytes(b"")
        self.rejected("empty")
        self.path.unlink()
        with self.writer():
            pass
        self.assertEqual(inspect(self.path)["event_count"], 0)

    def test_unknown_schema_rejected(self):
        self.demo()
        self.mutate(lambda rows: rows[0].update(schema="sp-vision-events-2"))
        self.rejected("unsupported schema")

    def test_host_time_backwards_rejected(self):
        with self.writer() as writer:
            self.pose(writer, host=100)
            with self.assertRaisesRegex(RecordingError, "host monotonic"):
                self.pose(writer, host=99, source=101)
            self.pose(writer, host=101, source=101)
        self.assertEqual(len(list(Reader(self.path))), 2)

    def test_source_and_mapping_backwards_rejected(self):
        with self.writer() as writer:
            self.pose(writer, host=200, source=100, mapped=100)
            with self.assertRaisesRegex(RecordingError, "moved backwards"):
                self.pose(writer, host=201, source=99, mapped=101)
            with self.assertRaisesRegex(RecordingError, "moved backwards"):
                self.pose(writer, host=201, source=101, mapped=99)

    def test_unmapped_source_is_not_silently_assumed_to_be_host(self):
        with self.writer() as writer:
            self.pose(writer, source=5_000_000, host=100)
        report = inspect(self.path)
        self.assertEqual(report["unmapped_events"], 1)
        self.assertEqual(report["ingress_delay_us"], {})

    def test_clock_reset_needs_explicit_epoch_and_cannot_return_to_old_epoch(self):
        with self.writer() as writer:
            self.pose(writer, host=100, source=50_000)
            with self.assertRaisesRegex(RecordingError, "epoch"):
                self.pose(writer, host=101, source=1, epoch="boot-2")
            self.epoch(writer)
            self.pose(writer, host=102, source=1, epoch="boot-2")
            with self.assertRaisesRegex(RecordingError, "epoch"):
                self.pose(writer, host=103, source=50_001, epoch="boot-1")
            with self.assertRaisesRegex(RecordingError, "epoch"):
                self.epoch(writer, host=103, previous="boot-2", current="boot-1")
        self.assertEqual(inspect(self.path)["event_counts"]["clock_epoch"], 1)

    def test_pose_brackets_do_not_join_across_boots(self):
        with self.writer() as writer:
            self.pose(writer, host=90, source=9000, mapped=90)
            self.image(writer, host=100, measured=100)
            self.epoch(writer, host=105)
            self.pose(writer, host=110, source=1, epoch="boot-2", mapped=110)
        brackets = inspect(self.path)["image_pose_brackets"]["by_sensor_frames_and_epoch"]
        self.assertEqual(len(brackets), 2)
        self.assertEqual([x["images_with_pose_bracket_at_most_50ms"] for x in brackets], [0, 0])

    def test_pose_brackets_do_not_join_different_sensors(self):
        with self.writer() as writer:
            self.pose(writer, host=90, source=90, mapped=90, sensor="small_y")
            self.image(writer, host=100, measured=100)
            self.pose(writer, host=110, source=110, mapped=110, sensor="big_y")
        brackets = inspect(self.path)["image_pose_brackets"]["by_sensor_frames_and_epoch"]
        self.assertEqual([x["images_with_pose_bracket_at_most_50ms"] for x in brackets], [0, 0])

    def test_high_uncertainty_pose_excluded_from_brackets(self):
        self.demo()
        self.mutate(lambda rows: [row["stamp"].update(uncertainty_us=6000)
                                 for row in rows if row["kind"] == "pose"])
        report = inspect(self.path)
        self.assertEqual(report["mapping_uncertainty_over_5ms_events"], 12)
        self.assertEqual(report["image_pose_brackets"]["by_sensor_frames_and_epoch"], [])

    def test_mapping_pair_and_host_identity_checked(self):
        self.demo()
        original = self.path.read_bytes()
        self.mutate(lambda rows: rows[1]["stamp"].update(uncertainty_us=None))
        self.rejected("present together")
        self.path.write_bytes(original)
        self.mutate(lambda rows: rows[3]["stamp"].update(mapped_host_us=1_000_999))
        self.rejected("identity mapping")

    def test_future_mapped_sample_rejected(self):
        with self.writer() as writer:
            with self.assertRaisesRegex(RecordingError, "later than recording"):
                self.pose(writer, host=100, mapped=101)

    def test_duplicate_source_timestamp_reported(self):
        with self.writer() as writer:
            self.pose(writer, host=100, source=1)
            self.pose(writer, host=101, source=1)
        self.assertEqual(inspect(self.path)["duplicate_source_timestamps"], 1)

    def test_detection_requires_image_and_image_measurement_time(self):
        self.demo()
        original = self.path.read_bytes()
        self.mutate(lambda rows: rows[3]["data"].update(image_id="not-recorded"))
        self.rejected("earlier image")
        self.path.write_bytes(original)
        self.mutate(lambda rows: rows[3]["stamp"].update(time_us=1_002_000, mapped_host_us=1_002_000))
        self.rejected("measurement times disagree")

    def test_invalid_quaternion_not_normalized_silently(self):
        self.demo()
        self.mutate(lambda rows: rows[1]["data"].update(quaternion_wxyz=[2, 0, 0, 0]))
        self.rejected("already be normalized")

    def test_feedback_cannot_reference_unknown_or_other_session_command(self):
        self.demo()
        original = self.path.read_bytes()
        self.mutate(lambda rows: rows[6]["data"].update(applied_command_id="unknown"))
        self.rejected("missing prior command")
        self.path.write_bytes(original)
        self.mutate(lambda rows: rows[6]["data"].update(session_id=2))
        self.rejected("session mismatch")

    def test_reference_mismatch_not_used_as_tracking_error(self):
        self.demo()
        self.mutate(lambda rows: [row["data"].update(reference="world_los")
                                 for row in rows if row["kind"] == "feedback"])
        report = inspect(self.path)["commands"]
        self.assertEqual(report["reference_mismatch_feedback_samples"], 11)
        self.assertIsNone(report["yaw_feedback_minus_command_rad"])

    def test_feedback_age_is_sample_time_not_receive_time(self):
        self.demo()
        report = inspect(self.path)["commands"]
        self.assertEqual(report["feedback_sample_minus_generation_us"]["min"], 2000)
        self.assertAlmostEqual(report["yaw_feedback_minus_command_rad"]["median"], -.005)

    def test_reader_output_mutation_does_not_change_validation_state(self):
        self.demo()
        count = 0
        for event in Reader(self.path):
            if event["kind"] == "command":
                event["data"]["session_id"] = 1000
            count += 1
        self.assertEqual(count, 73)

    def test_writer_detaches_input_and_rejected_append_does_not_consume_sequence(self):
        with self.writer() as writer:
            ts = stamp(100)
            payload = {"component": "test", "status": "ok", "detail": "example"}
            writer.append("health", stream="health", host_us=100, timestamp=ts, data=payload)
            payload["status"] = "bad"
            ts["time_us"] = 0
            with self.assertRaises(RecordingError):
                writer.append("health", stream="health", host_us=101, timestamp=stamp(101), data=payload)
            payload["status"] = "ok"
            writer.append("health", stream="health", host_us=102, timestamp=stamp(102), data=payload)
        events = list(Reader(self.path))
        self.assertEqual([x["seq"] for x in events], [1, 2])
        self.assertEqual(events[0]["data"]["status"], "ok")

    def test_closed_writer_refuses_more_records_and_images(self):
        writer = self.writer()
        writer.close()
        with self.assertRaisesRegex(RecordingError, "closed"):
            self.pose(writer)
        with self.assertRaisesRegex(RecordingError, "closed"):
            writer.write_image("one.pgm", b"")

    def test_cli_readonly_replay_and_external_report(self):
        self.demo()
        before = self.path.read_bytes()
        report = self.root / "logs" / "report.json"
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(["replay", str(self.path)]), 0)
            self.assertEqual(main(["inspect", str(self.path), "--report", str(report)]), 0)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(json.loads(report.read_text())["integrity"], "passed")
        with redirect_stderr(io.StringIO()):
            self.assertEqual(main(["inspect", str(self.path), "--report", str(self.path)]), 2)
        self.assertEqual(self.path.read_bytes(), before)

    def test_corruption_cli_returns_failure_without_report(self):
        self.demo()
        (self.path.parent / "images/image-0000.pgm").unlink()
        report = self.root / "logs" / "report.json"
        with redirect_stderr(io.StringIO()):
            self.assertEqual(main(["inspect", str(self.path), "--report", str(report)]), 2)
        self.assertFalse(report.exists())


if __name__ == "__main__":
    unittest.main()
