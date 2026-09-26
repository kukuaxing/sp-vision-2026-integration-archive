"""Command line tools for immutable capture fixtures and read-only event playback."""

import argparse
import json
import sys
from pathlib import Path

from .demo import make_demo
from .recording import Reader, RecordingError, inspect, replay_digest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline recording integrity and read-only event replay; no devices")
    sub = parser.add_subparsers(dest="command", required=True)
    demo = sub.add_parser("demo", help="Create 12 synthetic PGM frames and all event types; refuses overwrite")
    demo.add_argument("--output", type=Path, default=Path("data/replay/demo/events.jsonl"))
    demo.add_argument("--config", type=Path, help="Embed this config snapshot verbatim; does not use it to operate hardware")
    check = sub.add_parser("inspect", help="Validate complete recording and referenced images; output integrity report")
    check.add_argument("recording", type=Path)
    check.add_argument("--report", type=Path, help="Write generated JSON report, conventionally under logs/replay/")
    replay = sub.add_parser("replay", help="Deterministically consume events without sleep, algorithm execution or control")
    replay.add_argument("recording", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "demo":
            path = make_demo(args.output, args.config)
            result = {"synthetic_only": True, "recording": str(path), "report": inspect(path)}
        elif args.command == "inspect":
            result = inspect(args.recording)
            if args.report:
                # A report must never replace the input recording or its checked image payloads.
                recording_root = args.recording.resolve().parent
                report_path = args.report.resolve()
                if report_path.is_relative_to(recording_root):
                    raise RecordingError("report must be outside recording directory (use logs/replay/)")
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        else:
            result = replay_digest(Reader(args.recording))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (RecordingError, OSError) as exc:
        print(f"RECORDING REJECTED: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
