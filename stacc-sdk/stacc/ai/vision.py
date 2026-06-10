"""Universal computer vision — perception pipeline for any robot.

Pluggable backends: OpenCV (default), ONNX Runtime, PyTorch.
Falls back gracefully if a backend is unavailable.

Modules:
- ObjectDetector: YOLO/RT-DETR via ONNX, or color/contour based via OpenCV
- FeatureExtractor: ORB, SIFT, AKAZE feature points
- OpticalFlow: Lucas-Kanade and Farneback dense flow
- ImageSegmentation: color-based, threshold, contour
- DepthEstimator: stereo disparity, monocular depth
- VisualOdometry: feature-based camera motion estimation
- ColorDetector: find objects by color in HSV space
- ArucoDetector: detect ArUco/AprilTag markers for localization

Usage:
    from stacc.ai.vision import ObjectDetector, FeatureExtractor, ColorDetector

    detector = ObjectDetector(backend="opencv")
    detections = detector.detect(image)

    features = FeatureExtractor(method="orb")
    keypoints, descriptors = features.extract(image)
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


# ══════════════════════════════════════════
# DETECTION RESULT
# ══════════════════════════════════════════


class Detection:
    """Single object detection result."""

    __slots__ = ("label", "confidence", "bbox", "center", "area", "mask")

    def __init__(
        self,
        label: str = "",
        confidence: float = 0.0,
        bbox: Optional[List[int]] = None,
        center: Optional[List[float]] = None,
        area: float = 0.0,
        mask: Any = None,
    ):
        self.label = label
        self.confidence = confidence
        self.bbox = bbox or [0, 0, 0, 0]  # [x, y, w, h]
        self.center = center or [0.0, 0.0]
        self.area = area
        self.mask = mask

    def to_dict(self) -> Dict[str, Any]:
        return {
            "label": self.label,
            "confidence": round(self.confidence, 3),
            "bbox": self.bbox,
            "center": [round(c, 1) for c in self.center],
            "area": round(self.area, 1),
        }


# ══════════════════════════════════════════
# OBJECT DETECTOR
# ══════════════════════════════════════════


class ObjectDetector:
    """Universal object detector.

    Supports multiple backends:
    - "opencv": Color/contour based detection (no ML, always available)
    - "onnx": ONNX Runtime models (YOLO, RT-DETR, etc.)
    - "torch": PyTorch models

    Args:
        backend: Detection backend
        model_path: Path to ONNX/PyTorch model file
        confidence_threshold: Minimum detection confidence
        class_names: List of class names for the model
    """

    def __init__(
        self,
        backend: str = "opencv",
        model_path: Optional[str] = None,
        confidence_threshold: float = 0.5,
        class_names: Optional[List[str]] = None,
    ):
        self.backend = backend
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.class_names = class_names or []
        self._model = None

        if backend == "onnx" and model_path:
            self._load_onnx(model_path)
        elif backend == "torch" and model_path:
            self._load_torch(model_path)

    def _load_onnx(self, path: str):
        try:
            import onnxruntime as ort

            self._model = ort.InferenceSession(path)
        except ImportError:
            print("  [vision] onnxruntime not available, falling back to opencv")
            self.backend = "opencv"

    def _load_torch(self, path: str):
        try:
            import torch

            self._model = torch.load(path, map_location="cpu")
            self._model.eval()
        except ImportError:
            print("  [vision] torch not available, falling back to opencv")
            self.backend = "opencv"

    def detect(self, image) -> List[Detection]:
        if image is None:
            return []

        if self.backend == "onnx" and self._model:
            return self._detect_onnx(image)
        elif self.backend == "torch" and self._model:
            return self._detect_torch(image)
        else:
            return self._detect_opencv(image)

    def _detect_opencv(self, image) -> List[Detection]:
        if not HAS_CV2 or not HAS_NUMPY:
            return []

        detections = []
        gray = (
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        )
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cx, cy = x + w / 2, y + h / 2

            detections.append(
                Detection(
                    label="object",
                    confidence=min(area / 10000, 1.0),
                    bbox=[x, y, w, h],
                    center=[cx, cy],
                    area=area,
                )
            )

        return detections

    def _detect_onnx(self, image) -> List[Detection]:
        if not HAS_NUMPY:
            return []

        input_name = self._model.get_inputs()[0].name
        h, w = image.shape[:2]
        blob = cv2.resize(image, (640, 640)) if HAS_CV2 else image
        blob = blob.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[np.newaxis]

        outputs = self._model.run(None, {input_name: blob})
        return self._parse_yolo_output(outputs[0], w, h)

    def _detect_torch(self, image) -> List[Detection]:
        try:
            import torch

            tensor = (
                torch.from_numpy(image).float().permute(2, 0, 1).unsqueeze(0) / 255.0
            )
            with torch.no_grad():
                output = self._model(tensor)
            return []  # parse model-specific output
        except Exception:
            return []

    def _parse_yolo_output(self, output, orig_w, orig_h) -> List[Detection]:
        detections = []
        if not HAS_NUMPY:
            return detections

        for det in output[0]:
            conf = float(det[4])
            if conf < self.confidence_threshold:
                continue

            cx, cy, w, h = det[0:4]
            x = int((cx - w / 2) * orig_w / 640)
            y = int((cy - h / 2) * orig_h / 640)
            w = int(w * orig_w / 640)
            h = int(h * orig_h / 640)

            class_id = int(np.argmax(det[5:])) if len(det) > 5 else 0
            label = (
                self.class_names[class_id]
                if class_id < len(self.class_names)
                else f"class_{class_id}"
            )

            detections.append(
                Detection(
                    label=label,
                    confidence=conf,
                    bbox=[x, y, w, h],
                    center=[x + w / 2, y + h / 2],
                    area=w * h,
                )
            )

        return detections


# ══════════════════════════════════════════
# FEATURE EXTRACTOR
# ══════════════════════════════════════════


class FeatureExtractor:
    """Extract visual features from images.

    Methods: ORB (fast, free), SIFT (accurate, patented in older OpenCV),
    AKAZE (good balance), BRISK (fast binary).

    Usage:
        fe = FeatureExtractor(method="orb", max_features=500)
        keypoints, descriptors = fe.extract(image)
        matches = fe.match(desc1, desc2)
    """

    def __init__(self, method: str = "orb", max_features: int = 500):
        self.method = method
        self.max_features = max_features
        self._detector = None
        self._matcher = None

        if HAS_CV2:
            if method == "orb":
                self._detector = cv2.ORB_create(max_features)
                self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            elif method == "akaze":
                self._detector = cv2.AKAZE_create()
                self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            elif method == "sift":
                self._detector = cv2.SIFT_create(max_features)
                self._matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
            elif method == "brisk":
                self._detector = cv2.BRISK_create()
                self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def extract(self, image) -> Tuple[Any, Any]:
        if self._detector is None or image is None:
            return [], None

        gray = (
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        )
        keypoints, descriptors = self._detector.detectAndCompute(gray, None)
        return keypoints, descriptors

    def match(self, desc1, desc2, ratio_threshold: float = 0.75) -> List[Any]:
        if self._matcher is None or desc1 is None or desc2 is None:
            return []
        try:
            matches = self._matcher.match(desc1, desc2)
            matches = sorted(matches, key=lambda m: m.distance)
            return matches[: self.max_features]
        except Exception:
            return []


# ══════════════════════════════════════════
# OPTICAL FLOW
# ══════════════════════════════════════════


class OpticalFlow:
    """Compute optical flow between consecutive frames.

    Methods:
    - "sparse": Lucas-Kanade (track specific points)
    - "dense": Farneback (flow for every pixel)

    Usage:
        flow = OpticalFlow(method="sparse")
        flow.update(frame)
        vectors = flow.get_flow()
    """

    def __init__(self, method: str = "sparse", max_points: int = 200):
        self.method = method
        self.max_points = max_points
        self._prev_gray = None
        self._prev_points = None
        self._flow = None

        self._lk_params = (
            dict(
                winSize=(15, 15),
                maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
            )
            if HAS_CV2
            else {}
        )

    def update(self, frame) -> Optional[Any]:
        if not HAS_CV2 or frame is None:
            return None

        gray = (
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        )

        if self._prev_gray is None:
            self._prev_gray = gray
            if self.method == "sparse":
                self._prev_points = cv2.goodFeaturesToTrack(
                    gray, self.max_points, 0.01, 10
                )
            return None

        if self.method == "sparse" and self._prev_points is not None:
            new_points, status, _ = cv2.calcOpticalFlowPyrLK(
                self._prev_gray, gray, self._prev_points, None, **self._lk_params
            )
            if new_points is not None:
                good_old = self._prev_points[status.flatten() == 1]
                good_new = new_points[status.flatten() == 1]
                self._flow = (good_old, good_new)
                self._prev_points = good_new.reshape(-1, 1, 2)
            else:
                self._prev_points = cv2.goodFeaturesToTrack(
                    gray, self.max_points, 0.01, 10
                )

        elif self.method == "dense":
            self._flow = cv2.calcOpticalFlowFarneback(
                self._prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0
            )

        self._prev_gray = gray
        return self._flow

    def get_flow(self) -> Optional[Any]:
        return self._flow

    def get_mean_motion(self) -> Tuple[float, float]:
        if self._flow is None:
            return 0.0, 0.0
        if self.method == "sparse" and isinstance(self._flow, tuple):
            old, new = self._flow
            if len(old) == 0:
                return 0.0, 0.0
            dx = float(np.mean(new[:, 0] - old[:, 0]))
            dy = float(np.mean(new[:, 1] - old[:, 1]))
            return dx, dy
        elif self.method == "dense" and HAS_NUMPY:
            return float(np.mean(self._flow[:, :, 0])), float(
                np.mean(self._flow[:, :, 1])
            )
        return 0.0, 0.0

    def reset(self):
        self._prev_gray = None
        self._prev_points = None
        self._flow = None


# ══════════════════════════════════════════
# COLOR DETECTOR
# ══════════════════════════════════════════


class ColorDetector:
    """Detect objects by color in HSV space.

    Simple but effective for colored objects (balls, cones, lines).

    Usage:
        cd = ColorDetector()
        cd.add_color("red", [0, 100, 100], [10, 255, 255])
        cd.add_color("blue", [100, 100, 100], [130, 255, 255])
        detections = cd.detect(image)
    """

    def __init__(self):
        self.colors: Dict[str, Tuple[List[int], List[int]]] = {}

    def add_color(self, name: str, hsv_lower: List[int], hsv_upper: List[int]):
        self.colors[name] = (hsv_lower, hsv_upper)

    def detect(self, image, min_area: int = 500) -> List[Detection]:
        if not HAS_CV2 or image is None:
            return []

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = []

        for color_name, (lower, upper) in self.colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                M = cv2.moments(contour)
                cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else x + w // 2
                cy = int(M["m01"] / M["m00"]) if M["m00"] > 0 else y + h // 2

                detections.append(
                    Detection(
                        label=color_name,
                        confidence=min(area / 5000, 1.0),
                        bbox=[x, y, w, h],
                        center=[cx, cy],
                        area=area,
                        mask=mask,
                    )
                )

        return detections


# ══════════════════════════════════════════
# ARUCO / APRILTAG DETECTOR
# ══════════════════════════════════════════


class ArucoDetector:
    """Detect ArUco markers for precise localization.

    ArUco markers provide 6-DOF pose estimation from a single camera.
    Used for robot localization, calibration, and navigation.

    Args:
        dictionary: ArUco dictionary type (4x4_50, 5x5_100, etc.)
        marker_size: Physical size of markers in meters
        camera_matrix: 3x3 camera intrinsic matrix
        dist_coeffs: Camera distortion coefficients
    """

    DICTIONARIES = {
        "4x4_50": cv2.aruco.DICT_4X4_50 if HAS_CV2 else 0,
        "4x4_100": cv2.aruco.DICT_4X4_100 if HAS_CV2 else 0,
        "5x5_50": cv2.aruco.DICT_5X5_50 if HAS_CV2 else 0,
        "5x5_100": cv2.aruco.DICT_5X5_100 if HAS_CV2 else 0,
        "6x6_50": cv2.aruco.DICT_6X6_50 if HAS_CV2 else 0,
    }

    def __init__(
        self,
        dictionary: str = "4x4_50",
        marker_size: float = 0.05,
        camera_matrix: Optional[Any] = None,
        dist_coeffs: Optional[Any] = None,
    ):
        self.marker_size = marker_size
        self._aruco_dict = None
        self._detector = None
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        if HAS_CV2:
            dict_id = self.DICTIONARIES.get(dictionary, cv2.aruco.DICT_4X4_50)
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, params)

    def detect(self, image) -> List[Dict[str, Any]]:
        if self._detector is None or image is None:
            return []

        gray = (
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        )
        corners, ids, rejected = self._detector.detectMarkers(gray)

        results = []
        if ids is None:
            return results

        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            center = [float(c[:, 0].mean()), float(c[:, 1].mean())]

            result = {
                "id": int(marker_id),
                "corners": c.tolist(),
                "center": center,
            }

            if self.camera_matrix is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i : i + 1],
                    self.marker_size,
                    self.camera_matrix,
                    self.dist_coeffs,
                )
                result["position"] = tvecs[0][0].tolist()
                result["rotation"] = rvecs[0][0].tolist()
                result["distance"] = float(np.linalg.norm(tvecs[0][0]))

            results.append(result)

        return results


# ══════════════════════════════════════════
# VISUAL ODOMETRY
# ══════════════════════════════════════════


class VisualOdometry:
    """Estimate camera/robot motion from consecutive frames.

    Uses feature matching between frames to compute the relative
    transformation (rotation + translation).

    Args:
        method: Feature method ("orb", "akaze", "sift")
        camera_matrix: 3x3 intrinsic matrix (required for metric scale)
    """

    def __init__(self, method: str = "orb", camera_matrix: Optional[Any] = None):
        self.feature_extractor = FeatureExtractor(method=method, max_features=1000)
        self.camera_matrix = camera_matrix
        self._prev_kp = None
        self._prev_desc = None
        self._total_R = np.eye(3) if HAS_NUMPY else None
        self._total_t = np.zeros((3, 1)) if HAS_NUMPY else None
        self._trajectory = []

    def update(self, frame) -> Optional[Dict[str, Any]]:
        if not HAS_CV2 or not HAS_NUMPY or frame is None:
            return None

        kp, desc = self.feature_extractor.extract(frame)

        if self._prev_desc is None:
            self._prev_kp = kp
            self._prev_desc = desc
            return None

        matches = self.feature_extractor.match(self._prev_desc, desc)

        if len(matches) < 8:
            self._prev_kp = kp
            self._prev_desc = desc
            return None

        pts1 = np.float32([self._prev_kp[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp[m.trainIdx].pt for m in matches])

        if self.camera_matrix is not None:
            E, mask = cv2.findEssentialMat(
                pts1,
                pts2,
                self.camera_matrix,
                method=cv2.RANSAC,
                prob=0.999,
                threshold=1.0,
            )
            if E is not None:
                _, R, t, mask2 = cv2.recoverPose(E, pts1, pts2, self.camera_matrix)
                self._total_R = R @ self._total_R
                self._total_t = self._total_t + self._total_R @ t
        else:
            print(
                "  [vo] No camera_matrix provided. Provide a calibrated camera matrix "
                "for metric 3D visual odometry. Falling back to 2D homography "
                "(x, y only, no scale)."
            )
            H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)
            if H is not None:
                dx = float(H[0, 2])
                dy = float(H[1, 2])
                self._total_t[0, 0] += dx
                self._total_t[1, 0] += dy

        self._prev_kp = kp
        self._prev_desc = desc

        pose = [
            float(self._total_t[0, 0]),
            float(self._total_t[1, 0]),
            float(self._total_t[2, 0]) if len(self._total_t) > 2 else 0.0,
        ]
        self._trajectory.append(pose)

        return {
            "position": pose,
            "num_matches": len(matches),
            "num_inliers": int(mask.sum()) if mask is not None else 0,
        }

    @property
    def position(self) -> List[float]:
        if self._total_t is not None:
            return [float(self._total_t[i, 0]) for i in range(3)]
        return [0, 0, 0]

    @property
    def trajectory(self) -> List[List[float]]:
        return list(self._trajectory)

    def reset(self):
        self._prev_kp = None
        self._prev_desc = None
        self._total_R = np.eye(3) if HAS_NUMPY else None
        self._total_t = np.zeros((3, 1)) if HAS_NUMPY else None
        self._trajectory = []


# ══════════════════════════════════════════
# DEPTH ESTIMATION
# ══════════════════════════════════════════


class StereoDepthEstimator:
    """Compute depth from stereo camera pair.

    Uses OpenCV's StereoSGBM for semi-global block matching.

    Args:
        baseline: Camera baseline distance in meters
        focal_length: Focal length in pixels
        num_disparities: Number of disparity levels (must be divisible by 16)
        block_size: Block matching window size (odd number)
    """

    def __init__(
        self,
        baseline: float = 0.12,
        focal_length: float = 500.0,
        num_disparities: int = 64,
        block_size: int = 9,
    ):
        self.baseline = baseline
        self.focal_length = focal_length
        self._stereo = None

        if HAS_CV2:
            self._stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=num_disparities,
                blockSize=block_size,
                P1=8 * 3 * block_size**2,
                P2=32 * 3 * block_size**2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
            )

    def compute(self, left_image, right_image) -> Optional[Any]:
        if self._stereo is None or left_image is None or right_image is None:
            return None

        gray_l = (
            cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            if len(left_image.shape) == 3
            else left_image
        )
        gray_r = (
            cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            if len(right_image.shape) == 3
            else right_image
        )

        disparity = self._stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0
        disparity[disparity <= 0] = 0.1

        depth = (self.baseline * self.focal_length) / disparity
        return depth

    def compute_point_cloud(
        self, left_image, right_image
    ) -> Optional[List[List[float]]]:
        depth = self.compute(left_image, right_image)
        if depth is None:
            return None

        h, w = depth.shape
        points = []
        step = max(1, min(w, h) // 100)

        for v in range(0, h, step):
            for u in range(0, w, step):
                z = float(depth[v, u])
                if z <= 0 or z > 100:
                    continue
                x = (u - w / 2) * z / self.focal_length
                y = (v - h / 2) * z / self.focal_length
                points.append([x, y, z])

        return points
