#!/usr/bin/env python3
"""Export the alignment YOLO weights as a target-specific FP16 TensorRT engine."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a fixed-shape, batch-1 FP16 TensorRT engine. Run this "
            "script on the Jetson that will execute the engine."
        )
    )
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--workspace", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not source.is_file() or source.suffix.lower() != ".pt":
        raise ValueError("--source must reference an existing .pt model")
    if output.suffix.lower() != ".engine":
        raise ValueError("--output must use the .engine suffix")
    if args.input_size <= 0 or args.input_size % 32 != 0:
        raise ValueError("--input-size must be a positive multiple of 32")
    if args.workspace <= 0.0:
        raise ValueError("--workspace must be positive")

    from ultralytics import YOLO

    exported = Path(
        YOLO(str(source), task="detect").export(
            format="engine",
            imgsz=args.input_size,
            half=True,
            dynamic=False,
            batch=1,
            device=args.device,
            workspace=args.workspace,
            nms=False,
            verbose=False,
        )
    ).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if exported != output:
        shutil.move(str(exported), str(output))
    print(output)


if __name__ == "__main__":
    main()
