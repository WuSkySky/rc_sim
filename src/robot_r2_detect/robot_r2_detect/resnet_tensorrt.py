"""TensorRT ResNet classifier with no PyTorch or PyCUDA dependency."""

from __future__ import annotations

import ctypes
import math
from pathlib import Path

import cv2
import numpy as np


CUDA_SUCCESS = 0
CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2


class _CudaRuntime:
    """Minimal CUDA Runtime API used for TensorRT buffer management."""

    def __init__(self) -> None:
        self._lib = self._load_library()
        self._lib.cudaGetErrorString.argtypes = [ctypes.c_int]
        self._lib.cudaGetErrorString.restype = ctypes.c_char_p
        self._lib.cudaMalloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
        ]
        self._lib.cudaMalloc.restype = ctypes.c_int
        self._lib.cudaFree.argtypes = [ctypes.c_void_p]
        self._lib.cudaFree.restype = ctypes.c_int
        self._lib.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self._lib.cudaMemcpyAsync.restype = ctypes.c_int
        self._lib.cudaStreamCreate.argtypes = [
            ctypes.POINTER(ctypes.c_void_p)
        ]
        self._lib.cudaStreamCreate.restype = ctypes.c_int
        self._lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self._lib.cudaStreamSynchronize.restype = ctypes.c_int
        self._lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self._lib.cudaStreamDestroy.restype = ctypes.c_int

    @staticmethod
    def _load_library():
        for library_name in (
            "libcudart.so",
            "libcudart.so.12",
            "libcudart.so.11.0",
        ):
            try:
                return ctypes.CDLL(library_name)
            except OSError:
                continue
        raise RuntimeError(
            "CUDA Runtime library was not found; install JetPack/CUDA"
        )

    def _check(self, status: int, operation: str) -> None:
        if status == CUDA_SUCCESS:
            return
        detail = self._lib.cudaGetErrorString(status)
        message = (
            detail.decode("utf-8", errors="replace")
            if detail is not None
            else "unknown CUDA error"
        )
        raise RuntimeError(
            f"{operation} failed with CUDA error {status}: {message}"
        )

    def malloc(self, size: int) -> ctypes.c_void_p:
        pointer = ctypes.c_void_p()
        self._check(
            self._lib.cudaMalloc(ctypes.byref(pointer), size),
            "cudaMalloc",
        )
        return pointer

    def free(self, pointer: ctypes.c_void_p) -> None:
        if pointer.value:
            self._check(self._lib.cudaFree(pointer), "cudaFree")

    def create_stream(self) -> ctypes.c_void_p:
        stream = ctypes.c_void_p()
        self._check(
            self._lib.cudaStreamCreate(ctypes.byref(stream)),
            "cudaStreamCreate",
        )
        return stream

    def copy_async(
        self,
        destination: ctypes.c_void_p,
        source: ctypes.c_void_p,
        size: int,
        direction: int,
        stream: ctypes.c_void_p,
    ) -> None:
        self._check(
            self._lib.cudaMemcpyAsync(
                destination,
                source,
                size,
                direction,
                stream,
            ),
            "cudaMemcpyAsync",
        )

    def synchronize(self, stream: ctypes.c_void_p) -> None:
        self._check(
            self._lib.cudaStreamSynchronize(stream),
            "cudaStreamSynchronize",
        )

    def destroy_stream(self, stream: ctypes.c_void_p) -> None:
        if stream.value:
            self._check(
                self._lib.cudaStreamDestroy(stream),
                "cudaStreamDestroy",
            )


def validate_model_configuration(
    image_size: int,
    class_names: tuple[str, ...],
    mean: tuple[float, ...],
    std: tuple[float, ...],
) -> None:
    """Validate preprocessing and class metadata kept outside the engine."""
    if isinstance(image_size, bool) or image_size <= 0:
        raise ValueError("model_input_size must be a positive integer")
    if not class_names or any(not name for name in class_names):
        raise ValueError("model_class_names must contain non-empty names")
    if len(set(class_names)) != len(class_names):
        raise ValueError("model_class_names must be unique")
    if len(mean) != 3 or len(std) != 3:
        raise ValueError("model_mean and model_std must each contain 3 values")
    if not all(math.isfinite(value) for value in mean + std):
        raise ValueError("model_mean and model_std must be finite")
    if not all(value > 0.0 for value in std):
        raise ValueError("model_std values must be positive")


def preprocess_resnet_image(
    image: np.ndarray,
    image_size: int,
    mean: tuple[float, ...],
    std: tuple[float, ...],
) -> np.ndarray:
    """Apply training-time Resize(short side), CenterCrop and Normalize."""
    if not isinstance(image, np.ndarray) or image.ndim != 3:
        raise TypeError("KFS image must be an HxWxC numpy array")
    if image.shape[2] != 3 or image.shape[0] == 0 or image.shape[1] == 0:
        raise ValueError("KFS image must be a non-empty 3-channel image")

    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    resize_short = int(round(image_size / 0.875))
    source_height, source_width = rgb.shape[:2]
    scale = resize_short / min(source_height, source_width)
    resized_width = max(image_size, int(round(source_width * scale)))
    resized_height = max(image_size, int(round(source_height * scale)))
    resized = cv2.resize(
        rgb,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    left = (resized_width - image_size) // 2
    top = (resized_height - image_size) // 2
    cropped = resized[top:top + image_size, left:left + image_size]

    normalized = cropped.astype(np.float32) / 255.0
    normalized = (
        normalized - np.asarray(mean, dtype=np.float32).reshape(1, 1, 3)
    ) / np.asarray(std, dtype=np.float32).reshape(1, 1, 3)
    return np.ascontiguousarray(normalized.transpose(2, 0, 1))


def softmax(logits: np.ndarray) -> np.ndarray:
    """Return stable float32 probabilities for a batch of logits."""
    values = np.asarray(logits, dtype=np.float32)
    if values.ndim != 2:
        raise ValueError(f"Expected 2D logits, got shape {values.shape}")
    shifted = values - values.max(axis=1, keepdims=True)
    exponential = np.exp(shifted)
    return exponential / exponential.sum(axis=1, keepdims=True)


class TensorRTResNetClassifier:
    """Own a TensorRT engine/context and classify BGR OpenCV images."""

    def __init__(
        self,
        engine_path: Path,
        image_size: int,
        class_names: tuple[str, ...],
        mean: tuple[float, ...],
        std: tuple[float, ...],
    ) -> None:
        validate_model_configuration(image_size, class_names, mean, std)
        if not engine_path.is_file():
            raise FileNotFoundError(
                f"TensorRT engine not found: {engine_path}"
            )

        try:
            import tensorrt as trt
        except ImportError as exc:
            raise RuntimeError(
                "Python TensorRT bindings are unavailable; install "
                "JetPack package python3-libnvinfer"
            ) from exc

        self._trt = trt
        self._cuda = _CudaRuntime()
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        self._engine = self._runtime.deserialize_cuda_engine(
            engine_path.read_bytes()
        )
        if self._engine is None:
            raise RuntimeError(
                "TensorRT could not deserialize the engine. Engines are "
                "bound to the TensorRT version and target GPU; rebuild it "
                f"on this device: {engine_path}"
            )
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("TensorRT execution context creation failed")

        inputs: list[str] = []
        outputs: list[str] = []
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            mode = self._engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                inputs.append(name)
            elif mode == trt.TensorIOMode.OUTPUT:
                outputs.append(name)
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError(
                "Expected one TensorRT input and one output, got "
                f"inputs={inputs}, outputs={outputs}"
            )
        self._input_name = inputs[0]
        self._output_name = outputs[0]
        input_shape = tuple(self._engine.get_tensor_shape(self._input_name))
        if (
            len(input_shape) != 4
            or input_shape[1] != 3
            or input_shape[2:] != (image_size, image_size)
        ):
            raise ValueError(
                f"Engine input shape {input_shape} does not match "
                f"NCHW image size {image_size}"
            )

        self._input_dtype = np.dtype(
            trt.nptype(self._engine.get_tensor_dtype(self._input_name))
        )
        self._output_dtype = np.dtype(
            trt.nptype(self._engine.get_tensor_dtype(self._output_name))
        )
        self._stream = self._cuda.create_stream()
        self._device_input = ctypes.c_void_p()
        self._device_output = ctypes.c_void_p()
        self._input_capacity = 0
        self._output_capacity = 0
        self._closed = False

        self.engine_path = engine_path
        self.image_size = image_size
        self.class_names = class_names
        self.mean = mean
        self.std = std
        self.backend = f"TensorRT {trt.__version__}/CUDA"

        warmup = np.zeros(
            (1, 3, image_size, image_size), dtype=np.float32
        )
        output = self.infer(warmup)
        if output.shape != (1, len(class_names)):
            self.close()
            raise ValueError(
                f"Engine output shape {output.shape} does not match "
                f"{len(class_names)} configured classes"
            )

    def _ensure_buffers(self, input_bytes: int, output_bytes: int) -> None:
        if input_bytes > self._input_capacity:
            self._cuda.free(self._device_input)
            self._device_input = self._cuda.malloc(input_bytes)
            self._input_capacity = input_bytes
        if output_bytes > self._output_capacity:
            self._cuda.free(self._device_output)
            self._device_output = self._cuda.malloc(output_bytes)
            self._output_capacity = output_bytes

    def infer(self, images: np.ndarray) -> np.ndarray:
        if self._closed:
            raise RuntimeError("TensorRT classifier is closed")
        inputs = np.ascontiguousarray(images, dtype=self._input_dtype)
        if inputs.ndim != 4 or inputs.shape[1:] != (
            3,
            self.image_size,
            self.image_size,
        ):
            raise ValueError(
                "Expected NCHW input shaped "
                f"N,3,{self.image_size},{self.image_size}; got {inputs.shape}"
            )
        if not self._context.set_input_shape(self._input_name, inputs.shape):
            raise ValueError(
                f"TensorRT optimization profile rejected {inputs.shape}"
            )
        output_shape = tuple(
            self._context.get_tensor_shape(self._output_name)
        )
        if any(dimension < 0 for dimension in output_shape):
            raise RuntimeError(
                f"TensorRT output shape was not resolved: {output_shape}"
            )
        output = np.empty(output_shape, dtype=self._output_dtype)
        self._ensure_buffers(inputs.nbytes, output.nbytes)
        self._context.set_tensor_address(
            self._input_name, self._device_input.value
        )
        self._context.set_tensor_address(
            self._output_name, self._device_output.value
        )
        self._cuda.copy_async(
            self._device_input,
            ctypes.c_void_p(inputs.ctypes.data),
            inputs.nbytes,
            CUDA_MEMCPY_HOST_TO_DEVICE,
            self._stream,
        )
        if not self._context.execute_async_v3(self._stream.value):
            raise RuntimeError("TensorRT execute_async_v3 failed")
        self._cuda.copy_async(
            ctypes.c_void_p(output.ctypes.data),
            self._device_output,
            output.nbytes,
            CUDA_MEMCPY_DEVICE_TO_HOST,
            self._stream,
        )
        self._cuda.synchronize(self._stream)
        return output

    def classify(self, image: np.ndarray) -> tuple[int, str, float]:
        tensor = preprocess_resnet_image(
            image,
            self.image_size,
            self.mean,
            self.std,
        )
        probabilities = softmax(self.infer(tensor[np.newaxis, ...]))[0]
        class_id = int(np.argmax(probabilities))
        return (
            class_id,
            self.class_names[class_id],
            float(probabilities[class_id]),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._cuda.free(self._device_input)
        self._cuda.free(self._device_output)
        self._cuda.destroy_stream(self._stream)
        self._device_input = ctypes.c_void_p()
        self._device_output = ctypes.c_void_p()
        self._stream = ctypes.c_void_p()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
