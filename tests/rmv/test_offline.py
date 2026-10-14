import math
from pathlib import Path
import struct
import tempfile
import unittest

from tools.rmv.protocol import Frame, encode, StreamDecoder, fixed_payload, ACK_REQUEST, self_test
from tools.rmv.simulator import Board, Result
from tools.rmv.client import Client, InMemoryTransport, Journal, inspect_journal, Rejected
from tools.rmv.pose import ClockSync, Pose, PoseHistory, slerp


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.board = Board()
        self.transport = InMemoryTransport(self.board)
        self.client = Client(self.transport)
        self.client.connect(123)
        self.client.synchronize()
        self.transport.advance(15_000)
        self.client.heartbeat()

    def active(self):
        self.client.mode(3)
        self.transport.advance(50_000)
        self.client.heartbeat()

    def raw_ack(self, frame):
        frames = self.client.exchange_frame(frame)
        ack = next(f for f in frames if f.msg_type == 0xFF)
        return struct.unpack("<IHHII", ack.payload)

    def aim_frame(self, **changes):
        values = dict(epoch=self.board.epoch, produced=self.board.now,
                      deadline=self.board.now + 80_000, reference=2, policy=0,
                      valid=1, yaw=.1, pitch=.05, yr=0., pr=0., ya=0., pa=0.)
        values.update(changes)
        return self.client.frame(0x0201, fixed_payload("AIM_SETPOINT", *values.values()), ACK_REQUEST)

    def test_crc_framing_reference(self):
        self_test()

    def test_initial_state_never_arms(self):
        self.assertEqual(self.board.mode, 0)
        self.assertIsNone(self.board.target)
        self.assertAlmostEqual(self.client.clock.estimate(self.client.now)[0], 4_000_000)
        self.assertEqual(self.client.epoch, 1)

    def test_hello_requires_ack_request_as_delivered_firmware(self):
        board = Board()
        decoder = StreamDecoder()
        hello = Frame(1, 1, 0, 1000000, fixed_payload("HELLO", 456, 0x1F, 1, 1, 0), 0)
        replies = decoder.feed(board.receive(encode(hello)))
        self.assertEqual(board.session, 0)
        self.assertEqual(replies[0].msg_type, 0xFF)
        self.assertEqual(struct.unpack("<IHHII", replies[0].payload)[2], Result.BAD_VALUE)
        client = Client(InMemoryTransport(board))
        client.connect(456)
        self.assertNotEqual(client.session, 0)

    def test_permission_and_applied_sequence_are_distinct(self):
        self.active()
        frame = self.aim_frame()
        # Direct processing, before the next virtual control tick.
        self.board.receive(encode(frame))
        self.assertEqual(self.board.last_accepted, frame.seq)
        self.assertEqual(self.board.last_applied, 0)
        self.board.advance(10_000)
        self.assertEqual(self.board.last_applied, frame.seq)
        self.assertGreater(self.board.angle[0], 0)

    def test_heartbeat_does_not_extend_target(self):
        self.active()
        self.client.aim(.2, .1)
        for _ in range(12):
            self.transport.advance(10_000)
            self.client.heartbeat()
        self.assertIsNone(self.board.target)
        self.assertNotEqual(self.board.session, 0)
        self.assertEqual(self.board.velocity, [0., 0.])

    def test_duplicate_target_cannot_renew_deadline(self):
        self.active()
        frame = self.aim_frame()
        self.assertEqual(self.raw_ack(frame)[2], Result.OK)
        deadline = self.board.target[2]
        self.transport.advance(20_000)
        self.assertEqual(self.raw_ack(frame)[2], Result.STALE)
        self.assertEqual(self.board.target[2], deadline)

    def test_stop_retry_does_not_undo_later_mode(self):
        self.active()
        stop = self.client.frame(0x0202, fixed_payload("STOP_REMOTE", 0, 0), ACK_REQUEST)
        self.assertEqual(self.raw_ack(stop)[2], Result.OK)
        self.client.mode(3)
        self.assertEqual(self.raw_ack(stop)[2], Result.STALE)
        self.assertEqual(self.board.mode, 3)

    def test_control_validation_matrix(self):
        self.active()
        cases = [({"yaw": float("nan")}, Result.BAD_VALUE),
                 ({"yaw": 5.}, Result.OUT_OF_RANGE),
                 ({"epoch": 9}, Result.CONFLICT),
                 ({"policy": 1}, Result.UNSUPPORTED),
                 ({"valid": 3, "yr": .1}, Result.UNSUPPORTED),
                 ({"produced": self.board.now - 60_000}, Result.STALE),
                 ({"deadline": self.board.now + 200_000}, Result.STALE)]
        for changes, expected in cases:
            with self.subTest(changes=changes):
                self.assertEqual(self.raw_ack(self.aim_frame(**changes))[2], expected)
                self.assertIsNone(self.board.target)

    def test_invalid_target_cancels(self):
        self.active()
        self.client.aim(.2, .1)
        self.client.aim(0., 0., valid=False)
        self.assertIsNone(self.board.target)

    def test_physical_takeover(self):
        self.active()
        self.client.aim(.2, .1)
        self.board.physical_permit = False
        before = list(self.board.angle)
        self.transport.advance(20_000)
        self.assertEqual(self.board.angle, before)
        self.assertEqual(self.board.mode, 1)
        self.assertEqual(self.raw_ack(self.aim_frame())[2], Result.WRONG_MODE)

    def test_expired_session_and_reboot_cannot_replay(self):
        self.active()
        old = self.aim_frame()
        self.transport.advance(600_000)
        self.assertEqual(self.raw_ack(old)[2], Result.BAD_SESSION)
        self.client.connect(124)
        self.assertEqual(self.board.mode, 0)
        self.assertNotEqual(self.board.session, old.session_id)
        self.board.reboot()
        self.assertEqual(self.raw_ack(old)[2], Result.BAD_SESSION)

    def test_new_nonce_cannot_take_active_control(self):
        self.active()
        intruder = Client(self.transport)
        with self.assertRaises(Rejected) as context:
            intruder.connect(999)
        self.assertEqual(context.exception.result, Result.BUSY)
        self.assertEqual(self.board.nonce, 123)

    def test_client_clears_session_on_explicit_rejection(self):
        self.transport.advance(600_000)
        with self.assertRaises(Rejected) as error:
            self.client.heartbeat()
        self.assertEqual(error.exception.result, Result.BAD_SESSION)
        self.assertEqual(self.client.session, 0)
        self.assertFalse(self.client.poses.samples)

    def test_client_requires_recent_control_permission(self):
        self.active()
        self.client.status_time -= 200_000
        with self.assertRaises(ValueError):
            self.client.aim(.1, .05)

    def test_parameter_save_throttle(self):
        self.client.set_parameters({0x1003: .2}, persist=True)
        with self.assertRaises(Rejected) as error:
            self.client.set_parameters({0x1103: .2}, persist=True)
        self.assertEqual(error.exception.result, Result.BUSY)
        self.assertEqual(self.board.parameters[0x1103].current, .4)

    def test_unknown_messages_ack_only_when_requested(self):
        frame = self.client.frame(0x7777, b"x")
        self.assertEqual(self.board.receive(encode(frame)), b"")
        frame = self.client.frame(0x7777, b"x", ACK_REQUEST)
        self.assertEqual(self.raw_ack(frame)[2], Result.UNSUPPORTED)

    def test_reserved_and_short_payload_do_not_change_mode(self):
        for payload, expected in ((b"x", Result.BAD_LENGTH),
                                  (fixed_payload("MODE_REQUEST", 3, 3, 1, 1), Result.BAD_VALUE)):
            frame = self.client.frame(0x0200, payload, ACK_REQUEST)
            self.assertEqual(self.raw_ack(frame)[2], expected)
            self.assertEqual(self.board.mode, 0)

    def test_homing_is_not_falsely_implemented(self):
        with self.assertRaises(Rejected) as error:
            self.client.request(0x0205, fixed_payload("SERVICE_REQUEST", 1, 1, 3, 0, 1))
        self.assertEqual(error.exception.result, Result.UNSUPPORTED)

    def test_lost_ack_retry_preserves_operation_sequence(self):
        original = self.transport.exchange
        def lose_reply(wire):
            original(wire)
            return b""
        self.transport.exchange = lose_reply
        with self.assertRaises(TimeoutError):
            self.client.mode(3)
        original_seq = self.client.last_request[0].seq
        self.assertEqual(self.board.mode, 3)
        self.transport.exchange = original
        self.client.retry_last_request()
        self.assertEqual(self.board.op_seq, original_seq)

    def test_lost_mode_ack_blocks_aim_and_survives_read_queries(self):
        original = self.transport.exchange
        def lose_ack(wire):
            return b"".join(encode(f) for f in StreamDecoder().feed(original(wire))
                            if f.msg_type != 0x00FF)
        self.transport.exchange = lose_ack
        with self.assertRaises(TimeoutError):
            self.client.mode(3)
        mode_seq = self.client.last_request[0].seq
        self.transport.exchange = original
        self.transport.advance(50_000)
        self.client.poll()
        self.client.heartbeat()
        self.client.parameter(0x1003)
        self.assertEqual(self.client.last_request[0].seq, mode_seq)
        with self.assertRaises(ValueError):
            self.client.aim(.1, .05)
        self.assertIsNone(self.board.target)
        self.client.retry_last_request()
        self.client.aim(.1, .05)
        self.assertIsNotNone(self.board.target)

    def test_stop_timeout_immediately_locks_local_control(self):
        self.active()
        self.client.aim(.1, .05)
        original = self.transport.exchange
        def lose_all(wire):
            original(wire)
            return b""
        self.transport.exchange = lose_all
        with self.assertRaises(TimeoutError):
            self.client.stop()
        with self.assertRaises(ValueError):
            self.client.aim(.1, .05)
        self.assertIsNone(self.client.status)
        self.transport.exchange = original
        self.client.retry_last_request()
        self.assertEqual(self.board.mode, 0)
        with self.assertRaises(ValueError):
            self.client.aim(.1, .05)

    def test_epoch_and_geometry_require_a_new_mode_ack(self):
        self.active()
        for field in ("epoch", "geometry_rev"):
            setattr(self.board, field, getattr(self.board, field) + 1)
            self.transport.advance(50_000)
            self.client.poll()
            self.transport.advance(50_000)
            self.client.poll()
            with self.assertRaises(ValueError):
                self.client.aim(.1, .05)
            self.client.mode(3)
            self.transport.advance(50_000)
            self.client.poll()
            self.client.aim(.1, .05)

    def test_dedicated_time_reply_filters_old_echo_before_correct_reply(self):
        original = self.transport.exchange
        def prepend_stale(wire):
            request = StreamDecoder().feed(wire)[0]
            valid = original(wire)
            if request.msg_type == 2:
                t1, = struct.unpack("<Q", request.payload)
                wrong = Frame(0x8002, 1, self.client.session, self.board.now,
                              fixed_payload("TIME_REPLY", t1 - 1, self.board.now, self.board.now), 2)
                short = Frame(0x8002, 2, self.client.session, self.board.now, request.payload, 2)
                return encode(wrong) + encode(short) + valid
            return valid
        self.transport.exchange = prepend_stale
        self.assertEqual(self.client.synchronize()[0], 4_000_000)

    def test_parameter_dedicated_replies_filter_old_key_or_page(self):
        original = self.transport.exchange
        def prepend_stale(wire):
            request = StreamDecoder().feed(wire)[0]
            valid = original(wire)
            if request.msg_type in (0x0300, 0x0301):
                response = next(f for f in StreamDecoder().feed(valid)
                                if f.msg_type == request.msg_type + 0x8000)
                wrong = Frame(response.msg_type, response.seq, response.session_id,
                              response.sender_time_us, b"\xfe\xff" + response.payload[2:], 2)
                return encode(wrong) + valid
            return valid
        self.transport.exchange = prepend_stale
        self.assertEqual(len(self.client.list_parameters()), 8)
        self.assertEqual(self.client.parameter(0x1003)["key"], 0x1003)

    def test_lost_commit_is_not_overwritten_by_reads_or_next_transaction(self):
        original = self.transport.exchange
        def lose_commit_ack(wire):
            request = StreamDecoder().feed(wire)[0]
            response = original(wire)
            if request.msg_type == 0x0303:
                return b"".join(encode(f) for f in StreamDecoder().feed(response)
                                if f.msg_type != 0x00FF)
            return response
        self.transport.exchange = lose_commit_ack
        with self.assertRaises(TimeoutError):
            self.client.set_parameters({0x1003: .2})
        commit = self.client.last_request[0]
        self.assertEqual(commit.msg_type, 0x0303)
        revision = self.board.param_rev
        self.transport.exchange = original
        self.client.parameter(0x1003)
        self.client.list_parameters()
        self.client.heartbeat()
        self.assertEqual(self.client.last_request[0], commit)
        with self.assertRaises(RuntimeError):
            self.client.set_parameters({0x1003: .3})
        self.assertEqual(self.client.transaction, 1)
        self.client.retry_last_request()
        self.assertEqual(self.board.param_rev, revision)
        self.client.set_parameters({0x1003: .3})
        self.assertEqual(self.board.param_rev, revision + 1)

    def test_poll_resolves_delayed_mode_ack(self):
        original = self.transport.exchange
        delayed = []
        def delay(wire):
            response = original(wire)
            delayed.append(response)
            return b""
        self.transport.exchange = delay
        with self.assertRaises(TimeoutError):
            self.client.mode(3)
        self.transport.exchange = lambda wire: delayed.pop() + original(wire)
        self.client.poll()
        self.transport.exchange = original
        self.transport.advance(50_000)
        self.client.poll()
        self.client.aim(.1, .05)
        self.assertIsNotNone(self.board.target)

    def test_wrong_session_bad_session_ack_cannot_override_valid_ack(self):
        original = self.transport.exchange
        def prepend_foreign_ack(wire):
            request = StreamDecoder().feed(wire)[0]
            wrong = Frame(0x00FF, 1, self.client.session + 99, self.board.now,
                          fixed_payload("ACK", request.seq, request.msg_type, Result.BAD_SESSION, 0, 1), 2)
            return encode(wrong) + original(wire)
        self.transport.exchange = prepend_foreign_ack
        session = self.client.session
        self.client.mode(3)
        self.assertEqual(self.client.session, session)

    def test_hello_ignores_unmarked_ack_and_rejects_session_reuse(self):
        original = self.transport.exchange
        def prepend_unmarked_ack(wire):
            request = StreamDecoder().feed(wire)[0]
            wrong = Frame(0x00FF, 1, 0, self.board.now,
                          fixed_payload("ACK", request.seq, 1, Result.BUSY, 0, 1), 0)
            return encode(wrong) + original(wire)
        self.transport.exchange = prepend_unmarked_ack
        self.client.connect(124)
        session = self.client.session
        self.transport.exchange = original
        def reuse_session(wire):
            frames = StreamDecoder().feed(original(wire))
            return b"".join(encode(Frame(f.msg_type, f.seq, session if f.msg_type == 0x8001 else f.session_id,
                                          f.sender_time_us, f.payload, f.flags)) for f in frames)
        self.transport.exchange = reuse_session
        with self.assertRaises(ValueError):
            self.client.connect(125)
        self.assertEqual(self.client.session, 0)

    def test_parameter_wire_boundary_and_abort(self):
        self.client.set_parameters({0x1003: .01})
        self.assertAlmostEqual(self.client.parameter(0x1003)["current"], .01)
        from tools.rmv.protocol import value_slot
        self.client.transaction += 1
        self.client.request(0x0302, struct.pack("<IIHHHBB8s", self.client.transaction,
                            self.client.param_revision, 1, 0, 0x1003, 4, 0, value_slot(4, .2)))
        self.client.abort_parameters()
        self.assertIsNone(self.board.staged)
        self.assertAlmostEqual(self.board.parameters[0x1003].current, .01)

    def test_sensor_loss_invalidates_host_pose(self):
        self.board.sensor_online = False
        self.transport.advance(20_000)
        self.client.poll()
        self.assertIsNone(self.client.epoch)
        self.assertFalse(self.client.poses.samples)

    def test_crc_error_does_not_execute(self):
        self.active()
        raw = bytearray(encode(self.aim_frame()))
        raw[-1] ^= 1
        self.assertEqual(self.board.receive(raw), b"")
        self.assertIsNone(self.board.target)
        self.assertEqual(self.board.decoder.crc_errors, 1)

    def test_partial_and_concatenated_requests(self):
        request = self.client.frame(0x0301, struct.pack("<HH", 0x1003, 0))
        raw = encode(request)
        response = b""
        for byte in raw:
            response += self.board.receive(bytes([byte]))
        self.assertEqual(len(StreamDecoder().feed(response)), 1)
        response = self.board.receive(raw + raw)
        self.assertEqual(len(StreamDecoder().feed(response)), 2)

    def test_parameter_enumeration_and_atomic_rejection(self):
        self.assertEqual(len(self.client.list_parameters()), 8)
        initial = self.client.parameter(0x1003)["current"]
        with self.assertRaises(Rejected):
            self.client.set_parameters({0x1003: .2, 0x1103: 100.})
        self.assertAlmostEqual(self.client.parameter(0x1003)["current"], initial)
        self.assertEqual(self.board.param_rev, 1)

    def test_parameter_ram_persist_and_reboot(self):
        self.client.set_parameters({0x1003: .25}, persist=True)
        self.client.set_parameters({0x1103: .15})
        self.board.reboot()
        self.assertAlmostEqual(self.board.parameters[0x1003].current, .25)
        self.assertAlmostEqual(self.board.parameters[0x1103].current, .4)

    def test_failed_save_has_no_partial_commit(self):
        self.board.fail_save = True
        before = dict(self.board.persisted)
        with self.assertRaises(Rejected) as error:
            self.client.set_parameters({0x1003: .2}, persist=True)
        self.assertEqual(error.exception.result, Result.HARDWARE_FAULT)
        self.assertEqual(self.board.persisted, before)
        self.assertEqual(self.board.parameters[0x1003].current, .4)
        self.assertEqual(self.board.param_rev, 1)

    def test_parameter_revision_conflict(self):
        from tools.rmv.protocol import value_slot
        payload = struct.pack("<IIHHHBB8s", 1, 999, 1, 0, 0x1003, 4, 0, value_slot(4, .2))
        with self.assertRaises(Rejected) as error:
            self.client.request(0x0302, payload)
        self.assertEqual(error.exception.result, Result.CONFLICT)

    def test_duplicate_commit_does_not_increment_revision(self):
        self.client.set_parameters({0x1003: .2})
        revision = self.board.param_rev
        self.client.request(0x0303, fixed_payload("PARAM_COMMIT", 1, 1, 0, b"\0" * 3))
        self.assertEqual(self.board.param_rev, revision)

    def test_parameter_write_rejected_while_controlling(self):
        self.active()
        with self.assertRaises(Rejected) as error:
            self.client.set_parameters({0x1003: .2})
        self.assertEqual(error.exception.result, Result.WRONG_MODE)

    def test_expired_stage_cannot_commit(self):
        from tools.rmv.protocol import value_slot
        self.client.request(0x0302, struct.pack("<IIHHHBB8s", 1, 1, 1, 0, 0x1003, 4, 0, value_slot(4, .2)))
        for _ in range(22):
            self.transport.advance(100_000)
            self.client.heartbeat()
        with self.assertRaises(Rejected) as error:
            self.client.request(0x0303, fixed_payload("PARAM_COMMIT", 1, 1, 0, b"\0" * 3))
        self.assertEqual(error.exception.result, Result.NOT_READY)

    def test_subscription_budget_atomic(self):
        before = dict(self.board.subscriptions)
        payload = struct.pack("<B3sHHIHHI", 2, b"\0" * 3, 0x0102, 200, 1, 0x0103, 200, 0)
        with self.assertRaises(Rejected) as error:
            self.client.request(4, payload)
        self.assertEqual(error.exception.result, Result.RESOURCE_LIMIT)
        self.assertEqual(self.board.subscriptions, before)

    def test_optional_features_explicitly_unsupported(self):
        with self.assertRaises(Rejected) as error:
            self.client.request(0x0203, fixed_payload("SHOOT_REQUEST", 1, 0, 0, 0, 0, self.board.now + 10_000))
        self.assertEqual(error.exception.result, Result.UNSUPPORTED)

    def test_journal_roundtrip(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "wire.jsonl"
            journal = Journal(path)
            self.client.journal = journal
            self.client.heartbeat()
            self.client.parameter(0x1003)
            journal.close()
            report = inspect_journal(path)
            self.assertEqual(report["crc_errors"], 0)
            self.assertEqual(report["messages"]["rx:0x8301"], 1)


class PoseTests(unittest.TestCase):
    def test_shortest_arc_and_sign_equivalence(self):
        q = (math.cos(.3), 0, 0, math.sin(.3))
        self.assertAlmostEqual(abs(sum(a * b for a, b in zip(q, slerp(q, tuple(-v for v in q), .5)))), 1)
        a = (math.cos(math.radians(179) / 2), 0, 0, math.sin(math.radians(179) / 2))
        b = (math.cos(math.radians(-179) / 2), 0, 0, math.sin(math.radians(-179) / 2))
        self.assertAlmostEqual(abs(slerp(a, b, .5)[3]), 1)

    def test_history_epoch_gap_staleness_and_duplicates(self):
        history = PoseHistory()
        history.add(Pose(100_000, 1, 1, (1, 0, 0, 0)), 100_000)
        history.add(Pose(110_000, 2, 1, (1, 0, 0, 0)), 110_000)
        self.assertEqual(history.at(105_000, 115_000), (1, 0, 0, 0))
        for stamp, now in ((99_000, 115_000), (111_000, 115_000), (105_000, 211_000)):
            with self.assertRaises(ValueError):
                history.at(stamp, now)
        with self.assertRaises(ValueError):
            history.add(Pose(120_000, 2, 1, (1, 0, 0, 0)), 120_000)
        history.add(Pose(130_000, 1, 2, (1, 0, 0, 0)), 130_000)
        self.assertEqual(len(history.samples), 1)
        with self.assertRaises(ValueError):
            history.add(Pose(120_000, 3, 1, (1, 0, 0, 0)), 135_000)

    def test_invalid_norm_and_long_gap(self):
        history = PoseHistory()
        with self.assertRaises(ValueError):
            history.add(Pose(1, 1, 1, (0, 0, 0, 0)), 1)
        history.add(Pose(1, 1, 1, (1, 0, 0, 0)), 1)
        history.add(Pose(60_001, 2, 1, (1, 0, 0, 0)), 60_001)
        with self.assertRaises(ValueError):
            history.at(30_000, 60_001)

    def test_clock_sync_expiry_and_uncertainty(self):
        sync = ClockSync()
        with self.assertRaises(ValueError):
            sync.estimate(0)
        for i in range(8):
            base = i * 10_000
            sync.observe(base, base + 4001000, base + 4001000, base + 2000)
        self.assertEqual(sync.to_mcu(80_000, 80_000), 4_080_000)
        with self.assertRaises(ValueError):
            sync.estimate(4_000_000)
        sync.observe(80000, 5000000, 5000000, 82000)
        with self.assertRaises(ValueError):
            sync.estimate(82000)


if __name__ == "__main__":
    unittest.main()
