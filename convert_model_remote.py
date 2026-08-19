#!/usr/bin/env python3
"""Convert Ultralytics checkpoints or ONNX models to TensorRT engines.

The TensorRT engine is always built on the machine running this script.  This
is important for Jetson because serialized engines are not portable across
TensorRT versions or GPU architectures.
"""

from __future__ import annotations

import argparse
import json
import shlex
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SUPPORTED_TASKS = ("auto", "detect", "segment", "classify", "pose", "obb")


@dataclass(frozen=True)
class BuildConfig:
    height: int
    width: int
    channels: int
    input_name: str
    min_batch: int
    opt_batch: int
    max_batch: int
    dynamic: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert one or more Ultralytics .pt/ONNX models to ONNX and/or "
            "TensorRT. Model tasks such as detect, segment, classify, pose and "
            "OBB are supported."
        )
    )
    parser.add_argument("models", nargs="+", type=Path, help="Input .pt or .onnx model(s)")
    parser.add_argument(
        "--task",
        choices=SUPPORTED_TASKS,
        default="auto",
        help="Ultralytics task override for .pt input; default: infer from checkpoint",
    )
    parser.add_argument(
        "--imgsz",
        nargs="+",
        type=int,
        metavar="SIZE",
        help="Input size: one value for square input or two values HEIGHT WIDTH",
    )
    parser.add_argument("--channels", type=int, help="Input channel count; default: 3")
    parser.add_argument("--input-name", help="ONNX input tensor name; default: images")
    parser.add_argument("--batch", type=int, help="Set min/opt/max batch to the same value")
    parser.add_argument("--min-batch", type=int, help="TensorRT profile minimum batch")
    parser.add_argument("--opt-batch", type=int, help="TensorRT profile optimum batch")
    parser.add_argument("--max-batch", type=int, help="TensorRT profile maximum batch")
    shape_group = parser.add_mutually_exclusive_group()
    shape_group.add_argument("--dynamic", action="store_true", help="Export/use a dynamic batch axis")
    shape_group.add_argument("--static", action="store_true", help="Force a static batch axis")
    parser.add_argument(
        "--precision",
        choices=("fp32", "fp16"),
        default="fp16",
        help="TensorRT compute precision; default: fp16",
    )
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset for .pt export")
    parser.add_argument("--device", default="cpu", help="Ultralytics export device; default: cpu")
    parser.add_argument(
        "--workspace",
        type=int,
        default=1024,
        metavar="MIB",
        help="TensorRT workspace pool size in MiB; default: 1024",
    )
    parser.add_argument("--metadata", type=Path, help="Optional legacy/model JSON metadata")
    parser.add_argument("--output-dir", type=Path, help="Directory for generated engine files")
    parser.add_argument(
        "--engine",
        type=Path,
        help="Exact engine output path (valid only when converting one model)",
    )
    parser.add_argument("--onnx-only", action="store_true", help="Stop after exporting .pt to ONNX")
    parser.add_argument("--no-simplify", action="store_true", help="Disable ONNX graph simplification")
    parser.add_argument("--force", action="store_true", help="Allow replacing an existing engine")
    parser.add_argument(
        "--trtexec",
        action="store_true",
        help="Use the legacy ONNX -> trtexec path for .pt inputs (metadata is not preserved)",
    )
    parser.add_argument(
        "--skip-verify",
        action="store_true",
        help="Skip post-export Ultralytics metadata and blank-frame validation",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate options and print planned commands without converting",
    )
    return parser.parse_args()


def load_metadata(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    if not path.is_file():
        raise FileNotFoundError(f"Metadata file not found: {path}")
    with path.open(encoding="utf-8") as stream:
        metadata = json.load(stream)
    if not isinstance(metadata, dict):
        raise ValueError(f"Metadata root must be a JSON object: {path}")
    return metadata


def image_size(cli_size: list[int] | None, metadata: dict[str, Any]) -> tuple[int, int]:
    values = cli_size if cli_size is not None else metadata.get("imgsz", [640, 640])
    if isinstance(values, int):
        values = [values]
    if not isinstance(values, (list, tuple)) or len(values) not in (1, 2):
        raise ValueError("--imgsz/metadata imgsz must contain one or two integers")
    height = int(values[0])
    width = height if len(values) == 1 else int(values[1])
    if height <= 0 or width <= 0:
        raise ValueError("Image dimensions must be positive")
    return height, width


def build_config(args: argparse.Namespace, metadata: dict[str, Any]) -> BuildConfig:
    height, width = image_size(args.imgsz, metadata)
    profile = metadata.get("trt_profile", {})
    if not isinstance(profile, dict):
        raise ValueError("metadata trt_profile must be a JSON object")

    base_batch = args.batch
    min_batch = (
        args.min_batch
        if args.min_batch is not None
        else base_batch if base_batch is not None else int(profile.get("min_batch", 1))
    )
    opt_batch = (
        args.opt_batch
        if args.opt_batch is not None
        else base_batch if base_batch is not None else int(profile.get("opt_batch", min_batch))
    )
    max_batch = (
        args.max_batch
        if args.max_batch is not None
        else base_batch if base_batch is not None else int(profile.get("max_batch", opt_batch))
    )
    if not 1 <= min_batch <= opt_batch <= max_batch:
        raise ValueError(
            "Batch profile must satisfy 1 <= min-batch <= opt-batch <= max-batch"
        )

    inferred_dynamic = bool(metadata.get("dynamic_batch", False)) or min_batch != max_batch
    dynamic = False if args.static else args.dynamic or inferred_dynamic
    if not dynamic and min_batch != max_batch:
        raise ValueError("A static model cannot use different min/opt/max batch values")

    channels = args.channels if args.channels is not None else int(metadata.get("channels", 3))
    if channels <= 0:
        raise ValueError("--channels must be positive")
    return BuildConfig(
        height=height,
        width=width,
        channels=channels,
        input_name=args.input_name or str(metadata.get("input_name", "images")),
        min_batch=min_batch,
        opt_batch=opt_batch,
        max_batch=max_batch,
        dynamic=dynamic,
    )


def export_onnx(
    checkpoint: Path, args: argparse.Namespace, config: BuildConfig
) -> Path:
    expected_path = checkpoint.with_suffix(".onnx")
    if args.dry_run:
        task = "auto-detect" if args.task == "auto" else args.task
        print(
            "[dry-run] Ultralytics ONNX export: "
            f"model={checkpoint} task={task} imgsz={config.height}x{config.width} "
            f"batch={config.opt_batch} dynamic={config.dynamic} opset={args.opset}"
        )
        return expected_path

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "Converting .pt requires Ultralytics and PyTorch. Install them on the "
            "export machine, or export ONNX elsewhere and pass the .onnx file on Orin."
        ) from exc

    model_kwargs: dict[str, Any] = {}
    if args.task != "auto":
        model_kwargs["task"] = args.task
    model = YOLO(str(checkpoint), **model_kwargs)
    exported = model.export(
        format="onnx",
        imgsz=(config.height, config.width),
        batch=config.opt_batch,
        dynamic=config.dynamic,
        opset=args.opset,
        simplify=not args.no_simplify,
        device=args.device,
    )
    onnx_path = Path(exported)
    if not onnx_path.is_file():
        raise RuntimeError(f"Ultralytics did not create the expected ONNX file: {onnx_path}")
    print(f"ONNX created: {onnx_path}")
    return onnx_path


def locate_trtexec(dry_run: bool) -> str:
    executable = shutil.which("trtexec")
    jetpack_path = Path("/usr/src/tensorrt/bin/trtexec")
    if executable is None and jetpack_path.is_file():
        executable = str(jetpack_path)
    if executable is None:
        if dry_run:
            return "trtexec"
        raise RuntimeError(
            "trtexec was not found. Build the engine on a JetPack/TensorRT machine, "
            "or use --onnx-only on the export machine."
        )
    return executable


def engine_path_for(
    onnx_path: Path, args: argparse.Namespace, multiple_models: bool
) -> Path:
    if args.engine is not None:
        if multiple_models:
            raise ValueError("--engine cannot be used with multiple input models; use --output-dir")
        return args.engine
    directory = args.output_dir if args.output_dir is not None else onnx_path.parent
    return directory / f"{onnx_path.stem}_{args.precision}.engine"


def build_engine(
    onnx_path: Path,
    engine_path: Path,
    args: argparse.Namespace,
    config: BuildConfig,
) -> None:
    if not args.dry_run and not onnx_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {onnx_path}")
    if engine_path.exists() and not args.force and not args.dry_run:
        raise FileExistsError(f"Engine already exists (use --force to replace it): {engine_path}")

    trtexec = locate_trtexec(args.dry_run)
    command = [
        trtexec,
        f"--onnx={onnx_path}",
        f"--saveEngine={engine_path}",
        f"--memPoolSize=workspace:{args.workspace}",
        "--skipInference",
    ]
    if args.precision == "fp16":
        command.append("--fp16")
    if config.dynamic:
        shape = lambda batch: (  # noqa: E731 - compact TensorRT argument construction
            f"{config.input_name}:{batch}x{config.channels}x{config.height}x{config.width}"
        )
        command.extend(
            (
                f"--minShapes={shape(config.min_batch)}",
                f"--optShapes={shape(config.opt_batch)}",
                f"--maxShapes={shape(config.max_batch)}",
            )
        )

    print("Running:", shlex.join(command))
    if args.dry_run:
        return
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(command, check=True)
    print(f"TensorRT engine created: {engine_path}")


def export_engine_direct(
    checkpoint: Path,
    engine_path: Path,
    args: argparse.Namespace,
    config: BuildConfig,
) -> None:
    """Export a checkpoint directly so Ultralytics metadata stays in the plan."""

    if engine_path.exists() and not args.force:
        raise FileExistsError(f"Engine already exists (use --force to replace it): {engine_path}")

    try:
        import numpy as np
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "Direct TensorRT export requires the target Jetson Ultralytics/PyTorch install."
        ) from exc

    # Export from a temporary checkpoint path. This prevents Ultralytics from
    # touching a stale <checkpoint>.engine file before validation succeeds.
    with tempfile.TemporaryDirectory(prefix="ultralytics_engine_") as temp_dir:
        temp_checkpoint = Path(temp_dir) / checkpoint.name
        shutil.copy2(checkpoint, temp_checkpoint)
        model = YOLO(str(temp_checkpoint))
        actual_task = getattr(model, "task", None)
        if args.task != "auto" and actual_task != args.task:
            raise ValueError(
                f"Checkpoint task mismatch: requested {args.task!r}, "
                f"but {checkpoint} is {actual_task!r}."
            )
        if not isinstance(getattr(model, "names", None), dict) or not model.names:
            raise ValueError(f"Checkpoint has no class metadata: {checkpoint}")

        export_device = 0 if str(args.device).lower() == "cpu" else args.device
        export_kwargs = {
            "format": "engine",
            "imgsz": (config.height, config.width),
            "batch": config.opt_batch,
            "dynamic": config.dynamic,
            "device": export_device,
            "workspace": args.workspace,
            "nms": False,
            "verbose": False,
        }
        if args.precision == "fp16":
            # Ultralytics 8.4.x uses quantize=16; half=True is deprecated.
            export_kwargs["quantize"] = 16

        exported = Path(model.export(**export_kwargs)).resolve()
        if not exported.is_file():
            raise RuntimeError(f"Ultralytics did not create an engine: {exported}")

        if not args.skip_verify:
            loaded = YOLO(str(exported), task=actual_task)
            if dict(loaded.names) != dict(model.names):
                raise RuntimeError(
                    "Engine metadata mismatch: "
                    f"expected {dict(model.names)!r}, got {dict(loaded.names)!r}"
                )
            if getattr(loaded, "task", None) != actual_task:
                raise RuntimeError(
                    f"Engine loaded as task={getattr(loaded, 'task', None)!r}, "
                    f"expected {actual_task!r}"
                )
            loaded.predict(
                source=[np.zeros((config.height, config.width, 3), dtype=np.uint8)],
                imgsz=(config.height, config.width),
                conf=0.5,
                iou=0.7,
                max_det=1,
                device=export_device,
                verbose=False,
            )

        engine_path.parent.mkdir(parents=True, exist_ok=True)
        staged = engine_path.with_suffix(engine_path.suffix + ".tmp")
        if staged.exists():
            staged.unlink()
        shutil.copy2(exported, staged)

    staged.replace(engine_path)
    print(f"Validated TensorRT engine: {engine_path}")


def main() -> None:
    args = parse_args()
    if args.workspace <= 0:
        raise ValueError("--workspace must be positive")
    if args.opset <= 0:
        raise ValueError("--opset must be positive")

    metadata = load_metadata(args.metadata)
    config = build_config(args, metadata)
    multiple_models = len(args.models) > 1
    if args.engine is not None and multiple_models:
        raise ValueError("--engine cannot be used with multiple input models; use --output-dir")

    for model_path in args.models:
        suffix = model_path.suffix.lower()
        if not args.dry_run and not model_path.is_file():
            raise FileNotFoundError(f"Model not found: {model_path}")
        if suffix == ".pt":
            if not args.onnx_only and not args.trtexec:
                output_path = engine_path_for(
                    model_path.with_suffix(".onnx"), args, multiple_models
                )
                export_engine_direct(model_path, output_path, args, config)
                continue
            onnx_path = export_onnx(model_path, args, config)
        elif suffix == ".onnx":
            onnx_path = model_path
        else:
            raise ValueError(f"Unsupported model format {suffix!r}: {model_path} (expected .pt or .onnx)")

        if not args.onnx_only:
            output_path = engine_path_for(onnx_path, args, multiple_models)
            build_engine(onnx_path, output_path, args, config)


if __name__ == "__main__":
    main()
