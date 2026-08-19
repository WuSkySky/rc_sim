#!/usr/bin/env python3
"""Export and validate an alignment YOLO detection TensorRT engine.

The engine is exported directly by Ultralytics instead of converting an ONNX
file with ``trtexec``.  Direct export preserves the task and class metadata
that the runtime detector needs in order to decode the output tensor.
"""

from __future__ import annotations

import argparse
import gc
from pathlib import Path
import shutil
import tempfile
from typing import Any


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
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace an existing output engine after validation.",
    )
    parser.add_argument(
        "--skip-verify",
        action="store_true",
        help="Skip the post-export Ultralytics load/predict validation.",
    )
    return parser.parse_args()


def _load_detection_model(source: Path):
    from ultralytics import YOLO

    model = YOLO(str(source))
    task = getattr(model, "task", None)
    if task != "detect":
        model_type = type(getattr(model, "model", None)).__name__
        raise ValueError(
            "The alignment exporter requires a detection checkpoint, but "
            f"{source} is task={task!r} ({model_type}). Use the detection "
            "checkpoint, not a segmentation/classification checkpoint."
        )
    names = getattr(model, "names", None)
    if not isinstance(names, dict) or not names:
        raise ValueError(f"The detection checkpoint has no class metadata: {source}")
    return model


def _validate_engine(
    engine: Path,
    expected_names: dict[Any, Any],
    input_size: int,
    device: int,
) -> None:
    """Load the generated plan through the same Ultralytics API as the node."""

    import numpy as np
    from ultralytics import YOLO

    loaded = YOLO(str(engine), task="detect")
    actual_names = dict(getattr(loaded, "names", {}))
    if actual_names != dict(expected_names):
        raise RuntimeError(
            "Exported engine metadata does not match the checkpoint: "
            f"expected names={dict(expected_names)!r}, got {actual_names!r}."
        )
    if getattr(loaded, "task", None) != "detect":
        raise RuntimeError(
            f"Exported engine was loaded as task={getattr(loaded, 'task', None)!r}, "
            "not task='detect'."
        )

    # A blank-frame smoke test catches raw/incorrectly decoded outputs (for
    # example confidence values outside [0, 1]) before the engine is deployed.
    loaded.predict(
        source=[np.zeros((input_size, input_size, 3), dtype=np.uint8)],
        imgsz=input_size,
        conf=0.5,
        iou=0.7,
        max_det=1,
        device=device,
        verbose=False,
    )


def _memory_preflight() -> None:
    """Warn before a Jetson export competes with running ROS inference nodes."""

    try:
        mem_available_kib = next(
            int(line.split()[1])
            for line in Path("/proc/meminfo").read_text().splitlines()
            if line.startswith("MemAvailable:")
        )
    except (StopIteration, OSError, ValueError):
        return
    available_mib = mem_available_kib // 1024
    if available_mib < 3072:
        print(
            "WARNING: only "
            f"{available_mib} MiB RAM is available. Stop real1/ROS inference "
            "and remote IDE workers before exporting to avoid CUDA allocation failure."
        )


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
    if output.exists() and not args.force:
        raise FileExistsError(
            f"Output engine already exists: {output} (use --force to replace it)"
        )

    _memory_preflight()
    model = _load_detection_model(source)
    output.parent.mkdir(parents=True, exist_ok=True)

    # Export from a temporary checkpoint path so an existing engine or a stale
    # ``<checkpoint>.engine`` file is never overwritten before validation.
    with tempfile.TemporaryDirectory(prefix="r2_engine_export_") as temp_dir:
        temp_source = Path(temp_dir) / source.name
        shutil.copy2(source, temp_source)
        import onnx
        import torch
        from ultralytics.utils.export.engine import onnx2engine

        temp_model = type(model)(str(temp_source), task="detect")
        onnx_path = Path(
            temp_model.export(
                format="onnx",
                imgsz=(args.input_size, args.input_size),
                batch=1,
                dynamic=False,
                opset=17,
                simplify=True,
                device="cpu",
                verbose=False,
            )
        ).resolve()
        if not onnx_path.is_file():
            raise RuntimeError(f"Ultralytics did not create ONNX: {onnx_path}")
        onnx_model = onnx.load(str(onnx_path), load_external_data=False)
        metadata = {item.key: item.value for item in onnx_model.metadata_props}
        metadata.setdefault("task", "detect")
        metadata.setdefault("names", str(model.names))
        metadata.setdefault("imgsz", str([args.input_size, args.input_size]))
        metadata.setdefault("batch", "1")
        metadata.setdefault("channels", "3")
        metadata.setdefault("args", str({"dynamic": False, "nms": False}))
        expected_names = dict(model.names)
        del onnx_model, temp_model, model
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        exported = Path(temp_dir) / f"{source.stem}.engine"
        onnx2engine(
            str(onnx_path),
            output_file=exported,
            workspace=args.workspace,
            quantize=16,
            dynamic=False,
            shape=(1, 3, args.input_size, args.input_size),
            metadata=metadata,
            verbose=False,
            prefix="TensorRT:",
        )
        exported = exported.resolve()
        if not exported.is_file():
            raise RuntimeError(f"TensorRT did not create an engine: {exported}")
        if not args.skip_verify:
            _validate_engine(exported, expected_names, args.input_size, args.device)
        staged_output = output.with_suffix(output.suffix + ".tmp")
        if staged_output.exists():
            staged_output.unlink()
        shutil.copy2(exported, staged_output)
    staged_output.replace(output)
    print(f"Validated TensorRT engine: {output}")


if __name__ == "__main__":
    main()
