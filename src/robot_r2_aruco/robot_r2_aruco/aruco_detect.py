#!/usr/bin/env python3
"""ROS 2 node for ArUco marker detection and 6-DOF pose estimation."""

from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from robot_r2_interfaces.msg import ArucoMarkerPose, ArucoPoseDetection, CameraFrame
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from robot_r2_detect.camera_frame import (
    bgr_to_image_message,
    camera_frame_header,
    camera_frame_to_bgr,
    camera_qos,
)

_PARAMETER_NAMES = (
    "target_marker_id",
    "aruco_dictionary",
    "marker_size_mm",
    "adaptive_thresh_win_size_min",
    "adaptive_thresh_win_size_max",
    "adaptive_thresh_win_size_step",
    "adaptive_thresh_constant",
    "min_marker_perimeter_rate",
    "max_marker_perimeter_rate",
    "polygonal_approx_accuracy_rate",
    "min_corner_distance_rate",
    "min_marker_distance_rate",
    "marker_border_bits",
    "publish_tf",
    "axis_length_mm",
    "visualization_enabled",
    "target_processing_rate",
    "processing_width",
)
_RUNTIME_PARAMS = frozenset(_PARAMETER_NAMES)


def _param_qos() -> QoSProfile:
    """QoS for low-frequency parameter-like topics (e.g. CameraInfo)."""
    return QoSProfile(depth=10)


def _advance_rate_limit(
    now: float, next_allowed: float, period: float
) -> tuple[bool, float]:
    """Return whether to process now and the next allowed monotonic time."""
    if now < next_allowed:
        return False, next_allowed
    return True, now + period


def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w).

    Uses the trace-based method; assumes R is a valid orthonormal matrix.
    """
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (float(R[2, 1]) - float(R[1, 2])) / s
        y = (float(R[0, 2]) - float(R[2, 0])) / s
        z = (float(R[1, 0]) - float(R[0, 1])) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + float(R[0, 0]) - float(R[1, 1]) - float(R[2, 2])) * 2.0
        w = (float(R[2, 1]) - float(R[1, 2])) / s
        x = 0.25 * s
        y = (float(R[0, 1]) + float(R[1, 0])) / s
        z = (float(R[0, 2]) + float(R[2, 0])) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + float(R[1, 1]) - float(R[0, 0]) - float(R[2, 2])) * 2.0
        w = (float(R[0, 2]) - float(R[2, 0])) / s
        x = (float(R[0, 1]) + float(R[1, 0])) / s
        y = 0.25 * s
        z = (float(R[1, 2]) + float(R[2, 1])) / s
    else:
        s = math.sqrt(1.0 + float(R[2, 2]) - float(R[0, 0]) - float(R[1, 1])) * 2.0
        w = (float(R[1, 0]) - float(R[0, 1])) / s
        x = (float(R[0, 2]) + float(R[2, 0])) / s
        y = (float(R[1, 2]) + float(R[2, 1])) / s
        z = 0.25 * s
    return x, y, z, w


def _build_aruco_dictionary(dictionary_name: str):
    """Return an OpenCV ArUco dictionary for the given name.

    Raises ValueError if the dictionary name is unsupported.
    """
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV was built without the aruco module")
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(
            f"unsupported ArUco dictionary: {dictionary_name}"
        )
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def _build_detector_parameters(
    adaptive_thresh_win_size_min: int,
    adaptive_thresh_win_size_max: int,
    adaptive_thresh_win_size_step: int,
    adaptive_thresh_constant: float,
    min_marker_perimeter_rate: float,
    max_marker_perimeter_rate: float,
    polygonal_approx_accuracy_rate: float,
    min_corner_distance_rate: float,
    min_marker_distance_rate: float,
    marker_border_bits: int,
):
    """Build cv2.aruco.DetectorParameters with the given values."""
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = adaptive_thresh_win_size_min
    params.adaptiveThreshWinSizeMax = adaptive_thresh_win_size_max
    params.adaptiveThreshWinSizeStep = adaptive_thresh_win_size_step
    params.adaptiveThreshConstant = adaptive_thresh_constant
    params.minMarkerPerimeterRate = min_marker_perimeter_rate
    params.maxMarkerPerimeterRate = max_marker_perimeter_rate
    params.polygonalApproxAccuracyRate = polygonal_approx_accuracy_rate
    params.minCornerDistanceRate = min_corner_distance_rate
    params.minMarkerDistanceRate = min_marker_distance_rate
    params.markerBorderBits = marker_border_bits
    return params


class ArucoPoseDetector:
    """Own an OpenCV ArUco detector and estimate 6-DOF marker poses."""

    def __init__(
        self,
        dictionary_name: str,
        marker_size_mm: float,
        detector_params,
    ) -> None:
        self._dictionary = _build_aruco_dictionary(dictionary_name)
        self._parameters = detector_params
        self._detector = cv2.aruco.ArucoDetector(
            self._dictionary, self._parameters,
        )
        self._marker_size_mm = marker_size_mm
        half = marker_size_mm / 2.0
        self._object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )
        # Time-consistency state: reject a frame whose position jumps wildly
        # relative to the previous one (a marker-orientation flip). A moving
        # camera shifts tvec by at most ~0.01 m/frame at 0.3 m/s, so a 0.5 m
        # threshold only catches flips. The consecutive-reject reset keeps
        # genuine motion or marker re-detection from being locked out forever.
        self._last_tvec = None
        self._consecutive_rejects = 0

    def detect(self, image: np.ndarray) -> tuple[list, np.ndarray | None, int]:
        """Return (corners, ids, rejected_count) for all markers in a BGR image."""
        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        corners, ids, rejected = self._detector.detectMarkers(gray)
        self._refine_corners(gray, corners, ids)
        rejected_count = len(rejected) if rejected is not None else 0
        return corners, ids, rejected_count

    @staticmethod
    def _refine_corners(
        gray: np.ndarray,
        corners: list,
        ids: np.ndarray | None,
    ) -> None:
        """Refine marker corners to subpixel accuracy before pose estimation.

        ``detectMarkers`` returns integer-pixel corners (up to half a pixel
        of error); ``cornerSubPix`` sharpens them to ~0.1 px, which measurably
        improves the pose (especially the ``tvec`` distance) for small markers.
        """
        if ids is None:
            return
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001,
        )
        for corner in corners:
            cv2.cornerSubPix(gray, corner, (5, 5), (-1, -1), criteria)

    def estimate_pose(
        self,
        corners: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
        """Estimate the 6-DOF pose of a single marker.

        Returns (success, rvec, tvec).  tvec is in millimetres.

        Uses ``estimatePoseSingleMarkers`` so the IPPE square ambiguity
        (marker vs. its mirror) is resolved consistently instead of flipping
        between frames when the marker is nearly head-on.
        """
        marker_corners = np.asarray(corners, dtype=np.float32).reshape(1, 4, 2)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners,
            self._marker_size_mm,
            camera_matrix,
            dist_coeffs,
        )
        if rvecs is None or tvecs is None or len(tvecs) == 0:
            return False, None, None
        rvec = np.asarray(rvecs[0], dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvecs[0], dtype=np.float64).reshape(3, 1)
        # Prefer the solution where the marker is in front of the camera.
        if float(tvec[2]) < 0.0:
            return False, None, None

        # Reject an orientation flip: a frame whose position jumped wildly
        # versus the previous frame. 500 mm is far above normal frame-to-frame
        # motion (~0.01 m) and far below a flip (~2 m here).
        if self._last_tvec is not None:
            jump = float(np.linalg.norm(tvec - self._last_tvec))
            if jump > 500.0:  # millimetres
                self._consecutive_rejects += 1
                if self._consecutive_rejects <= 5:
                    return False, None, None
                self._consecutive_rejects = 0
            else:
                self._consecutive_rejects = 0
        self._last_tvec = tvec.copy()
        return True, rvec, tvec

    def draw_detections(
        self,
        image: np.ndarray,
        corners: list,
        ids: np.ndarray | None,
        poses: dict[int, tuple[np.ndarray, np.ndarray]],
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        axis_length_mm: float,
    ) -> np.ndarray:
        """Draw marker polygons, IDs, and 3-D coordinate axes onto a copy.

        ``poses`` maps marker_id → (rvec, tvec).  Entries are looked up by
        marker id rather than positional index, avoiding index-mismatch bugs
        when a target-marker-id filter is active.
        """
        visualization = image.copy()

        if ids is None:
            return visualization

        for i, marker_id in enumerate(ids.reshape(-1)):
            marker_id_int = int(marker_id)
            # Draw polygon outline in BGR orange.
            pts = np.asarray(corners[i], dtype=np.int32).reshape(-1, 2)
            cv2.polylines(
                visualization,
                [pts],
                isClosed=True,
                color=(0, 165, 255),
                thickness=2,
            )
            # Draw marker ID above the top-left corner.
            cv2.putText(
                visualization,
                f"ArUco {marker_id_int}",
                tuple(pts[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

            # Draw 3-D coordinate axes if a valid pose is available.
            rvec_tvec = poses.get(marker_id_int)
            if rvec_tvec is None:
                continue
            rvec, tvec = rvec_tvec

            axis_points_3d = np.array(
                [
                    [0.0, 0.0, 0.0],
                    [axis_length_mm, 0.0, 0.0],
                    [0.0, axis_length_mm, 0.0],
                    [0.0, 0.0, axis_length_mm],
                ],
                dtype=np.float32,
            )
            projected, _jacobian = cv2.projectPoints(
                axis_points_3d,
                rvec,
                tvec,
                camera_matrix,
                dist_coeffs,
            )
            origin = tuple(
                int(round(p)) for p in projected[0].ravel()
            )
            x_end = tuple(
                int(round(p)) for p in projected[1].ravel()
            )
            y_end = tuple(
                int(round(p)) for p in projected[2].ravel()
            )
            z_end = tuple(
                int(round(p)) for p in projected[3].ravel()
            )
            cv2.line(visualization, origin, x_end, (0, 0, 255), 2)
            cv2.line(visualization, origin, y_end, (0, 255, 0), 2)
            cv2.line(visualization, origin, z_end, (255, 0, 0), 2)

        return visualization


class ArucoDetectNode(Node):
    """Detect ArUco markers and publish their 6-DOF poses as TF frames."""

    def __init__(self) -> None:
        super().__init__("aruco_detect")
        self._state_lock = threading.Lock()
        self._parameter_lock = threading.Lock()
        self._image_callback_group = MutuallyExclusiveCallbackGroup()
        self._camera_info_callback_group = MutuallyExclusiveCallbackGroup()

        # Mutable state (protected by _state_lock).
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._camera_frame_id: str = ""
        self._next_processing_at = 0.0

        self._declare_parameters()
        self._load_parameters()

        image_qos = camera_qos()

        self._image_subscription = self.create_subscription(
            CameraFrame,
            "image_raw",
            self._on_image,
            image_qos,
            callback_group=self._image_callback_group,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            "camera_info",
            self._on_camera_info,
            _param_qos(),
            callback_group=self._camera_info_callback_group,
        )
        self._detection_publisher = self.create_publisher(
            ArucoPoseDetection, "detections", 10
        )
        self._debug_publisher = self.create_publisher(
            Image, "debug", image_qos
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        self.add_on_set_parameters_callback(self._on_parameters_changed)

    # ------------------------------------------------------------------
    # Parameter lifecycle
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("target_marker_id", -1)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_size_mm", 100.0)
        self.declare_parameter("adaptive_thresh_win_size_min", 3)
        self.declare_parameter("adaptive_thresh_win_size_max", 23)
        self.declare_parameter("adaptive_thresh_win_size_step", 10)
        self.declare_parameter("adaptive_thresh_constant", 7.0)
        self.declare_parameter("min_marker_perimeter_rate", 0.01)
        self.declare_parameter("max_marker_perimeter_rate", 4.0)
        self.declare_parameter("polygonal_approx_accuracy_rate", 0.05)
        self.declare_parameter("min_corner_distance_rate", 0.01)
        self.declare_parameter("min_marker_distance_rate", 0.01)
        self.declare_parameter("marker_border_bits", 1)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("axis_length_mm", 50.0)
        self.declare_parameter("visualization_enabled", False)
        self.declare_parameter("target_processing_rate", 30.0)
        self.declare_parameter("processing_width", 640)

    def _load_parameters(self) -> None:
        values = {
            name: self.get_parameter(name).value
            for name in _PARAMETER_NAMES
        }
        configuration, detector = self._validate_configuration(values)
        self._apply_configuration(configuration, detector)

    @staticmethod
    def _validate_configuration(values: dict) -> tuple[dict, ArucoPoseDetector]:
        """Validate a complete candidate configuration and build its detector."""
        integer_names = (
            "target_marker_id",
            "adaptive_thresh_win_size_min",
            "adaptive_thresh_win_size_max",
            "adaptive_thresh_win_size_step",
            "marker_border_bits",
            "processing_width",
        )
        for name in integer_names:
            if isinstance(values[name], bool) or not isinstance(values[name], int):
                raise ValueError(f"{name} must be an integer")

        number_names = (
            "marker_size_mm",
            "adaptive_thresh_constant",
            "min_marker_perimeter_rate",
            "max_marker_perimeter_rate",
            "polygonal_approx_accuracy_rate",
            "min_corner_distance_rate",
            "min_marker_distance_rate",
            "axis_length_mm",
            "target_processing_rate",
        )
        for name in number_names:
            value = values[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a number")
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")

        if not isinstance(values["aruco_dictionary"], str) or not values[
            "aruco_dictionary"
        ]:
            raise ValueError("aruco_dictionary must be a non-empty string")
        for name in ("publish_tf", "visualization_enabled"):
            if not isinstance(values[name], bool):
                raise ValueError(f"{name} must be a boolean")

        if values["target_marker_id"] < -1:
            raise ValueError("target_marker_id must be -1 or non-negative")
        if values["adaptive_thresh_win_size_min"] < 3:
            raise ValueError("adaptive_thresh_win_size_min must be at least 3")
        if (
            values["adaptive_thresh_win_size_max"]
            < values["adaptive_thresh_win_size_min"]
        ):
            raise ValueError(
                "adaptive_thresh_win_size_max must be greater than or equal "
                "to adaptive_thresh_win_size_min"
            )
        if values["adaptive_thresh_win_size_step"] <= 0:
            raise ValueError("adaptive_thresh_win_size_step must be positive")
        if values["marker_border_bits"] <= 0:
            raise ValueError("marker_border_bits must be positive")
        if values["processing_width"] < 0:
            raise ValueError("processing_width must be >= 0")

        positive_names = (
            "marker_size_mm",
            "min_marker_perimeter_rate",
            "max_marker_perimeter_rate",
            "polygonal_approx_accuracy_rate",
            "min_corner_distance_rate",
            "min_marker_distance_rate",
            "axis_length_mm",
            "target_processing_rate",
        )
        for name in positive_names:
            if float(values[name]) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if (
            float(values["max_marker_perimeter_rate"])
            < float(values["min_marker_perimeter_rate"])
        ):
            raise ValueError(
                "max_marker_perimeter_rate must be greater than or equal "
                "to min_marker_perimeter_rate"
            )

        configuration = dict(values)
        detector_params = _build_detector_parameters(
            **{
                name: configuration[name]
                for name in (
                    "adaptive_thresh_win_size_min",
                    "adaptive_thresh_win_size_max",
                    "adaptive_thresh_win_size_step",
                    "adaptive_thresh_constant",
                    "min_marker_perimeter_rate",
                    "max_marker_perimeter_rate",
                    "polygonal_approx_accuracy_rate",
                    "min_corner_distance_rate",
                    "min_marker_distance_rate",
                    "marker_border_bits",
                )
            }
        )
        detector = ArucoPoseDetector(
            dictionary_name=configuration["aruco_dictionary"],
            marker_size_mm=float(configuration["marker_size_mm"]),
            detector_params=detector_params,
        )
        return configuration, detector

    def _apply_configuration(
        self, configuration: dict, detector: ArucoPoseDetector
    ) -> None:
        rate = float(configuration["target_processing_rate"])
        with self._state_lock:
            self._configuration = configuration
            self._target_marker_id = configuration["target_marker_id"]
            self._visualization_enabled = configuration[
                "visualization_enabled"
            ]
            self._publish_tf = configuration["publish_tf"]
            self._axis_length_mm = float(configuration["axis_length_mm"])
            self._target_processing_rate = rate
            self._processing_width = configuration["processing_width"]
            self._processing_deadline_sec = 1.0 / rate
            self._processing_period_sec = 1.0 / rate
            self._next_processing_at = 0.0
            self._detector = detector

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        with self._parameter_lock:
            with self._state_lock:
                candidate = dict(self._configuration)
            for parameter in parameters:
                if parameter.name not in _RUNTIME_PARAMS:
                    return SetParametersResult(
                        successful=False,
                        reason=f"unknown parameter '{parameter.name}'",
                    )
                candidate[parameter.name] = parameter.value

            try:
                configuration, detector = self._validate_configuration(
                    candidate
                )
            except (RuntimeError, TypeError, ValueError) as exc:
                return SetParametersResult(
                    successful=False,
                    reason=str(exc),
                )

            self._apply_configuration(configuration, detector)

        changed = ", ".join(parameter.name for parameter in parameters)
        self.get_logger().info(f"updated ArUco parameters: {changed}")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_camera_info(self, message: CameraInfo) -> None:
        k = message.k
        if len(k) < 9:
            self.get_logger().warn(
                "camera_info.k has fewer than 9 elements; ignoring"
            )
            return
        camera_matrix = np.array(k[:9], dtype=np.float64).reshape(3, 3)
        if np.allclose(camera_matrix, 0.0):
            self.get_logger().warn(
                "camera_info.k is all zeros; ignoring"
            )
            return
        dist_coeffs = np.array(message.d, dtype=np.float64)
        with self._state_lock:
            self._camera_matrix = camera_matrix
            self._dist_coeffs = dist_coeffs
            self._camera_frame_id = message.header.frame_id
        # One-time info so the user knows the pipeline is live.
        if not getattr(self, "_camera_info_logged", False):
            self._camera_info_logged = True
            f_est = camera_matrix[0, 0]
            self.get_logger().info(
                f"camera_info received (fx≈{f_est:.0f} px); "
                f"aruco detection is now active"
            )

    def _on_image(self, message: CameraFrame) -> None:
        started_at = time.monotonic()

        with self._state_lock:
            period = self._processing_period_sec
            allowed, next_processing_at = _advance_rate_limit(
                started_at, self._next_processing_at, period
            )
            if not allowed:
                return
            self._next_processing_at = next_processing_at

        image: np.ndarray | None = None
        try:
            image = camera_frame_to_bgr(message)
        except Exception as exc:
            self.get_logger().debug(
                f"failed to convert CameraFrame: {exc}"
            )
            return

        with self._state_lock:
            camera_matrix = self._camera_matrix
            dist_coeffs = self._dist_coeffs
            camera_frame_id = self._camera_frame_id
            detector = self._detector
            target_marker_id = self._target_marker_id
            visualization_enabled = self._visualization_enabled
            publish_tf = self._publish_tf
            axis_length_mm = self._axis_length_mm
            processing_width = self._processing_width
            deadline_sec = self._processing_deadline_sec
            target_processing_rate = self._target_processing_rate

        if camera_matrix is None:
            self.get_logger().info(
                "waiting for camera_info", throttle_duration_sec=5.0
            )
            return

        # Optional downscale for detection on high-resolution feeds. The
        # source image is retained only as a view; detected corners are
        # scaled back so pose estimation stays in original pixel coordinates.
        original_image = image
        corner_scale = 1.0
        if processing_width > 0 and image.shape[1] > processing_width:
            corner_scale = float(image.shape[1]) / float(processing_width)
            target_height = max(1, int(round(image.shape[0] / corner_scale)))
            image = cv2.resize(
                image,
                (processing_width, target_height),
                interpolation=cv2.INTER_AREA,
            )

        # ---- Detect ----
        try:
            corners, ids, rejected_count = detector.detect(image)
        except Exception as exc:
            self.get_logger().warn(
                f"aruco detection failed: {exc}"
            )
            return

        # ---- Estimate pose per marker ----
        # Use dict {marker_id: (rvec, tvec)} so draw_detections can
        # look up poses by id rather than position — avoids the index
        # mismatch bug when target_marker_id filters some markers out.
        marker_poses: list[ArucoMarkerPose] = []
        poses: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        if ids is not None and corners is not None:
            for i, marker_id in enumerate(ids.reshape(-1)):
                marker_id_int = int(marker_id)
                if (
                    target_marker_id != -1
                    and marker_id_int != target_marker_id
                ):
                    continue

                try:
                    marker_corners = corners[i]
                    if corner_scale != 1.0:
                        marker_corners = np.asarray(
                            marker_corners, dtype=np.float32
                        ) * corner_scale
                    success, rvec, tvec = detector.estimate_pose(
                        marker_corners, camera_matrix, dist_coeffs
                    )
                except Exception as exc:
                    self.get_logger().debug(
                        f"pose estimation failed for marker "
                        f"{marker_id_int}: {exc}"
                    )
                    continue

                if not success:
                    continue

                poses[marker_id_int] = (rvec, tvec)

                # rvec → rotation matrix → quaternion
                R, _jacobian = cv2.Rodrigues(rvec)
                qx, qy, qz, qw = _rotation_matrix_to_quaternion(R)

                # tvec is in mm; convert to metres for ROS.
                tx = float(tvec[0]) / 1000.0
                ty = float(tvec[1]) / 1000.0
                tz = float(tvec[2]) / 1000.0

                marker_pose = ArucoMarkerPose()
                marker_pose.marker_id = marker_id_int
                marker_pose.pose.position.x = tx
                marker_pose.pose.position.y = ty
                marker_pose.pose.position.z = tz
                marker_pose.pose.orientation.x = qx
                marker_pose.pose.orientation.y = qy
                marker_pose.pose.orientation.z = qz
                marker_pose.pose.orientation.w = qw
                marker_poses.append(marker_pose)

                # Publish TF if enabled.
                if publish_tf and camera_frame_id:
                    self._publish_marker_tf(
                        message=message,
                        camera_frame_id=camera_frame_id,
                        marker_id=marker_id_int,
                        tx=tx,
                        ty=ty,
                        tz=tz,
                        qx=qx,
                        qy=qy,
                        qz=qz,
                        qw=qw,
                    )

        # Diagnostic: always log detection stats at INFO (throttled to 2 Hz).
        marker_count = len(marker_poses)
        detected_ids = sorted(poses.keys()) if ids is not None else None
        self.get_logger().info(
            f"aruco: detected={marker_count}, rejected={rejected_count}"
            + (f", ids={detected_ids}" if detected_ids else ""),
            throttle_duration_sec=2.0,
        )

        # ---- Publish detection message ----
        detection_msg = self._make_detection_message(
            message, marker_poses
        )
        self._detection_publisher.publish(detection_msg)

        # visualization_enabled is the sole debug-image switch. When false,
        # no debug image is copied, drawn, serialized, or published.
        if visualization_enabled and original_image is not None:
            if corner_scale != 1.0:
                # Draw on the downscaled detection image so the debug frame is
                # small and cheap to serialize; scale intrinsics to match.
                debug_image = image
                debug_corners = corners if corners is not None else []
                debug_matrix = camera_matrix.copy()
                debug_matrix[0, 0] /= corner_scale
                debug_matrix[1, 1] /= corner_scale
                debug_matrix[0, 2] /= corner_scale
                debug_matrix[1, 2] /= corner_scale
            else:
                debug_image = original_image
                debug_corners = corners if corners is not None else []
                debug_matrix = camera_matrix

            debug_image = detector.draw_detections(
                debug_image,
                debug_corners,
                ids,
                poses,
                debug_matrix,
                dist_coeffs,
                axis_length_mm,
            )

            status_text = f"markers: {marker_count}"
            if target_marker_id != -1:
                status_text += f" | target: {target_marker_id}"
            self._put_status_text(debug_image, status_text)
            debug_msg = bgr_to_image_message(
                debug_image, camera_frame_header(message)
            )
            self._debug_publisher.publish(debug_msg)

        # ---- Overrun warning ----
        processing_time = time.monotonic() - started_at
        if processing_time > deadline_sec:
            self.get_logger().warn(
                "aruco image processing overrun: "
                f"{processing_time * 1000.0:.2f} ms > "
                f"{deadline_sec * 1000.0:.2f} ms "
                f"(target {target_processing_rate:g} Hz)"
            )

    # ------------------------------------------------------------------
    # TF publishing
    # ------------------------------------------------------------------

    def _publish_marker_tf(
        self,
        message: CameraFrame,
        camera_frame_id: str,
        marker_id: int,
        tx: float,
        ty: float,
        tz: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        try:
            header = camera_frame_header(message)
        except ValueError as exc:
            self.get_logger().debug(
                f"skipping TF for marker {marker_id}: {exc}"
            )
            return
        transform = TransformStamped()
        transform.header.stamp = header.stamp
        transform.header.frame_id = camera_frame_id
        transform.child_frame_id = f"marker_{marker_id}"
        transform.transform.translation.x = tx
        transform.transform.translation.y = ty
        transform.transform.translation.z = tz
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

    # ------------------------------------------------------------------
    # Message helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_detection_message(
        source: CameraFrame,
        marker_poses: list[ArucoMarkerPose],
    ) -> ArucoPoseDetection:
        message = ArucoPoseDetection()
        try:
            message.header = camera_frame_header(source)
        except ValueError as exc:
            # This should not happen for a valid CameraFrame, but log it.
            message.header.frame_id = ""
            message.markers = marker_poses
            return message
        message.markers = marker_poses
        return message

    @staticmethod
    def _put_status_text(image: np.ndarray, text: str) -> None:
        cv2.putText(
            image,
            text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
