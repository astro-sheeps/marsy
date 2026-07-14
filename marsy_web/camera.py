"""
Camera helpers for the Marsy web dashboard.

The dashboard prefers Picamera2 on Raspberry Pi. If Picamera2 is unavailable,
it still serves a tiny placeholder JPEG so the web UI remains usable while the
rest of the rover controls are tested.

Camera rotation is applied in code before JPEG frames are sent to the browser.
This is intentionally not a CSS transform: downstream computer vision, saved
frames, map overlays, and the live view all see the same orientation.
"""

from __future__ import annotations

import base64
import io
import threading
import time
from dataclasses import dataclass
from typing import Iterator, Optional

# 1x1 black JPEG. Used when Picamera2 is unavailable or camera init fails.
_PLACEHOLDER_JPEG = base64.b64decode(
    b"/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAP//////////////////////////////////////////////////////////////////////////////////////"
    b"////////////////////////////////////////////2wBDAf//////////////////////////////////////////////////////////////////////////////////////"
    b"////////////////////////////////////////////wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAb/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIQAxAAAAH/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAEFAqf/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oACAEDAQE/Aaf/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oACAECAQE/Aaf/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAY/Aqf/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAE/IV//2gAMAwEAAgADAAAAEP/EABQRAQAAAAAAAAAAAAAAAAAAABD/2gAIAQMBAT8QH//EABQRAQAAAAAAAAAAAAAAAAAAABD/2gAIAQIBAT8QH//EABQQAQAAAAAAAAAAAAAAAAAAABD/2gAIAQEAAT8QH//Z"
)


def _normalise_rotation(degrees: int) -> int:
    """Return one of 0, 90, 180, 270."""
    value = int(degrees) % 360
    allowed = (0, 90, 180, 270)
    if value not in allowed:
        raise ValueError(f"camera rotation must be one of {allowed}, got {degrees!r}")
    return value


def _rotate_array_clockwise(array, rotation: int):
    """Rotate a numpy image array clockwise by 0/90/180/270 degrees."""
    rotation = _normalise_rotation(rotation)
    if rotation == 0:
        return array

    # Picamera2 depends on numpy, so this is available whenever capture_array()
    # works. np.rot90 uses counter-clockwise steps, therefore k is negative for
    # clockwise rotation.
    import numpy as np  # type: ignore

    if rotation == 90:
        return np.rot90(array, k=-1)
    if rotation == 180:
        return np.rot90(array, k=2)
    if rotation == 270:
        return np.rot90(array, k=1)
    return array


def _array_to_jpeg(array, quality: int = 85) -> bytes:
    """Encode a numpy RGB/BGR image array as JPEG using Pillow."""
    from PIL import Image  # type: ignore

    image = Image.fromarray(array)
    out = io.BytesIO()
    image.save(out, format="JPEG", quality=quality)
    return out.getvalue()


@dataclass
class CameraStatus:
    available: bool
    backend: str
    error: Optional[str] = None
    rotation_deg: int = 0
    rotation_backend: str = "none"
    frame_size: str = "unknown"


class CameraStream:
    """Small Picamera2 MJPEG frame provider."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: float = 8.0,
        rotation_deg: int = 90,
    ):
        self.width = width
        self.height = height
        self.fps = max(1.0, float(fps))
        self.rotation_deg = _normalise_rotation(rotation_deg)
        self._lock = threading.Lock()
        self._picam2 = None
        self._closed = threading.Event()
        self._status = CameraStatus(
            available=False,
            backend="placeholder",
            rotation_deg=self.rotation_deg,
            rotation_backend="software-array" if self.rotation_deg else "none",
            frame_size=f"{self.width}x{self.height}",
        )
        self._started = False

    @property
    def status(self) -> CameraStatus:
        return self._status

    def _configure_picamera2(self, picam2) -> None:
        # Keep the camera pipeline unrotated and rotate frames explicitly in
        # Python. This avoids libcamera Transform differences between Pi OS
        # versions and makes the behaviour predictable.
        controls = {"FrameRate": self.fps}
        config = picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            controls=controls,
        )
        picam2.configure(config)
        self._status.rotation_backend = "software-array" if self.rotation_deg else "none"
        self._status.frame_size = f"{self.width}x{self.height}"

    def start(self) -> None:
        with self._lock:
            if self._closed.is_set():
                return
            if self._started:
                return
            self._started = True
            try:
                from picamera2 import Picamera2  # type: ignore

                picam2 = Picamera2()
                self._configure_picamera2(picam2)
                picam2.start()
                time.sleep(0.4)
                self._picam2 = picam2
                self._status.available = True
                self._status.backend = "picamera2"
                self._status.error = None
            except Exception as exc:  # pragma: no cover - depends on Pi hardware
                self._picam2 = None
                self._status = CameraStatus(
                    available=False,
                    backend="placeholder",
                    error=f"{type(exc).__name__}: {exc}",
                    rotation_deg=self.rotation_deg,
                    rotation_backend="none",
                    frame_size="placeholder",
                )

    def stop(self) -> None:
        """Stop the camera permanently for this dashboard process.

        This is called during server shutdown. The flag is intentionally
        permanent: existing MJPEG request threads must not re-open Picamera2
        after cleanup has already run.
        """
        self._closed.set()
        with self._lock:
            picam2 = self._picam2
            self._picam2 = None
            self._started = False

        if picam2 is not None:
            try:
                picam2.stop()
            except Exception:
                pass
            try:
                close = getattr(picam2, "close", None)
                if callable(close):
                    close()
            except Exception:
                pass

        with self._lock:
            self._status = CameraStatus(
                available=False,
                backend="stopped",
                rotation_deg=self.rotation_deg,
                rotation_backend="software-array" if self.rotation_deg else "none",
                frame_size=f"{self.width}x{self.height}",
            )

    def capture_jpeg(self) -> bytes:
        if self._closed.is_set():
            return _PLACEHOLDER_JPEG
        self.start()
        if self._picam2 is None:
            return _PLACEHOLDER_JPEG

        try:
            array = self._picam2.capture_array("main")
            if self.rotation_deg:
                array = _rotate_array_clockwise(array, self.rotation_deg)
            data = _array_to_jpeg(array)
            if not data:
                return _PLACEHOLDER_JPEG
            return data
        except Exception as exc:  # pragma: no cover - depends on Pi hardware
            self._status.error = f"capture/rotation failed: {type(exc).__name__}: {exc}"
            return _PLACEHOLDER_JPEG

    def mjpeg_frames(self) -> Iterator[bytes]:
        interval = 1.0 / self.fps
        while not self._closed.is_set():
            frame = self.capture_jpeg()
            if self._closed.is_set():
                break
            yield (
                b"--marsyframe\r\n"
                b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                + frame
                + b"\r\n"
            )
            time.sleep(interval)
