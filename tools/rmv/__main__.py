"""Run from repository root: python -m tools.rmv demo | inspect PATH | vectors PATH."""
import argparse
import json
from pathlib import Path

from .client import Client, InMemoryTransport, Journal, inspect_journal
from .protocol import write_vectors
from .simulator import Board


def demo(path):
    board = Board()
    transport = InMemoryTransport(board)
    journal = Journal(path)
    try:
        client = Client(transport, journal)
        client.connect(12345)
        offset, uncertainty = client.synchronize()
        transport.advance(12_000)
        client.heartbeat()
        parameters = client.list_parameters()
        readback = client.set_parameters({0x1003: .3, 0x1103: .2})
        client.mode(3)
        transport.advance(50_000)
        client.heartbeat()
        for _ in range(40):
            client.aim(.12, .06)
            transport.advance(8_000)
            client.heartbeat()
        reached = list(board.angle)
        client.stop()
        before = list(board.angle)
        transport.advance(600_000)
        assert board.session == 0 and board.angle == before
        print(json.dumps({"environment": "SIMULATED ONLY", "boot": client.boot,
                          "offset_us": offset, "uncertainty_us": uncertainty,
                          "parameter_count": len(parameters), "readback": readback,
                          "virtual_angles_rad": reached, "stop_and_disconnect": "PASS",
                          "journal": str(path)}, indent=2))
    finally:
        journal.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("demo", help="in-process virtual board; never opens a serial port")
    run.add_argument("--journal", type=Path, default=Path("logs/rmv/offline_demo.jsonl"))
    inspect = commands.add_parser("inspect", help="decode a wire journal without sending anything")
    inspect.add_argument("path", type=Path)
    vectors = commands.add_parser("vectors")
    vectors.add_argument("path", type=Path)
    args = parser.parse_args()
    if args.command == "demo":
        demo(args.journal)
    elif args.command == "inspect":
        print(json.dumps(inspect_journal(args.path), indent=2))
    else:
        write_vectors(args.path)


if __name__ == "__main__":
    main()
