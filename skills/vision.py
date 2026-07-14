"""Pluggable camera and visual-detection backends for Marsy skills."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Protocol

from .models import Detection


class CapabilityUnavailable(RuntimeError):
    pass


class Detector(Protocol):
    def detect(self, image_path: Path, query: str) -> list[Detection]: ...


class CallableDetector:
    """Adapter for a local model, VLM API wrapper, or test callback."""

    def __init__(self, callback: Callable[[Path, str], Iterable[Detection | dict[str, Any]]]):
        self.callback = callback

    def detect(self, image_path: Path, query: str) -> list[Detection]:
        results: list[Detection] = []
        for item in self.callback(image_path, query):
            results.append(item if isinstance(item, Detection) else Detection.from_mapping(dict(item)))
        return results


class ArucoMarkerDetector:
    """Optional OpenCV ArUco detector. Requires an OpenCV build with cv2.aruco."""

    def __init__(self, dictionary_name: str = "DICT_4X4_50") -> None:
        self.dictionary_name = dictionary_name

    def detect(self, image_path: Path, query: str = "marker") -> list[Detection]:
        try:
            import cv2  # type: ignore
        except ImportError as exc:
            raise CapabilityUnavailable("OpenCV is required for ArUco marker detection") from exc

        if not hasattr(cv2, "aruco"):
            raise CapabilityUnavailable("This OpenCV build does not include cv2.aruco")

        image = cv2.imread(str(image_path))
        if image is None:
            raise RuntimeError(f"Could not read image: {image_path}")

        aruco = cv2.aruco
        dictionary_id = getattr(aruco, self.dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f"Unknown ArUco dictionary: {self.dictionary_name}")
        dictionary = aruco.getPredefinedDictionary(dictionary_id)

        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
            corners, ids, _ = detector.detectMarkers(image)
        else:
            corners, ids, _ = aruco.detectMarkers(image, dictionary)

        if ids is None:
            return []

        height, width = image.shape[:2]
        wanted_id: Optional[int] = None
        if ":" in query:
            _, raw_id = query.rsplit(":", 1)
            try:
                wanted_id = int(raw_id)
            except ValueError:
                wanted_id = None

        detections: list[Detection] = []
        for marker_corners, marker_id_array in zip(corners, ids):
            marker_id = int(marker_id_array[0])
            if wanted_id is not None and marker_id != wanted_id:
                continue
            points = marker_corners.reshape(-1, 2)
            x_min = max(0.0, float(points[:, 0].min()) / width)
            y_min = max(0.0, float(points[:, 1].min()) / height)
            x_max = min(1.0, float(points[:, 0].max()) / width)
            y_max = min(1.0, float(points[:, 1].max()) / height)
            detections.append(
                Detection(
                    label=f"marker:{marker_id}",
                    confidence=1.0,
                    bbox=[x_min, y_min, x_max, y_max],
                    marker_id=marker_id,
                )
            )
        return detections


class LocalCamera:
    """Single-frame Raspberry Pi camera capture with lazy optional dependencies."""

    def __init__(self, rotation_deg: Optional[int] = None, width: int = 640, height: int = 480) -> None:
        if rotation_deg is None:
            rotation_deg = int(os.getenv("MARSY_CAMERA_ROTATION", "90"))
        if rotation_deg not in {0, 90, 180, 270}:
            raise ValueError("rotation_deg must be one of 0, 90, 180, 270")
        self.rotation_deg = int(rotation_deg)
        self.width = int(width)
        self.height = int(height)

    def _rotate_file(self, path: Path) -> None:
        if self.rotation_deg == 0:
            return
        try:
            from PIL import Image  # type: ignore
        except ImportError as exc:
            raise CapabilityUnavailable("Pillow is required for camera rotation") from exc
        with Image.open(path) as image:
            image.rotate(-self.rotation_deg, expand=True).save(path, format="JPEG", quality=90)

    def _capture_picamera2(self, output_path: Path) -> bool:
        try:
            from picamera2 import Picamera2  # type: ignore
        except ImportError:
            return False

        camera = Picamera2()
        try:
            config = camera.create_still_configuration(main={"size": (self.width, self.height), "format": "RGB888"})
            camera.configure(config)
            camera.start()
            time.sleep(0.35)
            array = camera.capture_array()
            try:
                from PIL import Image  # type: ignore
            except ImportError as exc:
                raise CapabilityUnavailable("Pillow is required to save Picamera2 frames") from exc
            image = Image.fromarray(array)
            if self.rotation_deg:
                image = image.rotate(-self.rotation_deg, expand=True)
            image.save(output_path, format="JPEG", quality=90)
            return True
        finally:
            try:
                camera.stop()
            except Exception:
                pass
            close = getattr(camera, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass

    def _capture_cli(self, output_path: Path) -> bool:
        command = shutil.which("rpicam-still") or shutil.which("libcamera-still")
        if command is None:
            return False
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_file:
            temp_path = Path(temp_file.name)
        try:
            subprocess.run(
                [command, "-n", "-t", "1", "--width", str(self.width), "--height", str(self.height), "-o", str(temp_path)],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                timeout=15,
            )
            temp_path.replace(output_path)
            self._rotate_file(output_path)
            return True
        finally:
            temp_path.unlink(missing_ok=True)

    def capture(self, output_path: str | Path) -> Path:
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        errors: list[str] = []
        try:
            if self._capture_picamera2(output):
                return output
        except Exception as exc:
            errors.append(f"Picamera2: {type(exc).__name__}: {exc}")
        try:
            if self._capture_cli(output):
                return output
        except Exception as exc:
            errors.append(f"camera CLI: {type(exc).__name__}: {exc}")
        detail = "; ".join(errors)
        suffix = f" ({detail})" if detail else ""
        raise CapabilityUnavailable(
            "No usable Picamera2, rpicam-still, or libcamera-still camera backend is available" + suffix
        )


class VisionSystem:
    def __init__(
        self,
        camera: Optional[LocalCamera] = None,
        detector: Optional[Detector] = None,
        marker_detector: Optional[Detector] = None,
    ) -> None:
        self.camera = camera or LocalCamera()
        self.detector = detector
        self.marker_detector = marker_detector or ArucoMarkerDetector()

    def capture(self, output_path: str | Path) -> Path:
        return self.camera.capture(output_path)

    def detect(self, image_path: Path, query: str) -> list[Detection]:
        if query.strip().lower().startswith("marker"):
            return self.marker_detector.detect(image_path, query)
        if self.detector is None:
            raise CapabilityUnavailable(
                "No general visual detector is configured. Inject CallableDetector or a VLM adapter."
            )
        return self.detector.detect(image_path, query)

    def find_marker(self, image_path: Path, marker_id: Optional[int] = None) -> list[Detection]:
        query = "marker" if marker_id is None else f"marker:{marker_id}"
        return self.marker_detector.detect(image_path, query)
