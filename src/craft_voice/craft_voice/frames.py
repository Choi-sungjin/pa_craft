"""Camera frames for the voice node.

CRAFT cameras do not publish images over DDS. craft_sensors writes frames into
POSIX shared memory and announces each one on a frame_meta topic:

  /vision/d435/frame_meta  [offset, w, h, sequence, stamp_ns, depth_unit_nm]
      /dev/shm/craft_vision slot = BGR8 (w*h*3) + Z16 depth (w*h*2) + uint64 sequence
  /vision/c310/frame_meta  [offset, w, h, sequence, stamp_ns]
      /dev/shm/craft_c310   slot = BGR8 (w*h*3) + uint64 sequence

ShmFrameReader follows the same rules as craft_perception's SharedFrameReader:
shared non-blocking flock while copying, sequence trailer must match the
metadata, at most 4 slots. Only the color bytes are copied.
Plain ROS image messages (another robot, a bag, a stock TurtleBot camera) are
converted by image_to_bgr / compressed_to_bgr.
"""

from __future__ import annotations

import fcntl
import mmap
import os
import sys
from dataclasses import dataclass

import numpy as np

MAX_SLOTS = 4
SEQUENCE_BYTES = 8
MAX_SIDE = 8192
INT64_MAX = 2**63 - 1


@dataclass(frozen=True)
class Frame:
    bgr: np.ndarray
    sequence: int
    stamp_ns: int


def valid_meta(meta, depth: bool) -> bool:
    """Same checks as SharedFrameReader::is_valid_metadata."""
    if len(meta) != (6 if depth else 5):
        return False
    offset, width, height, _, stamp = (int(v) for v in meta[:5])
    if not (0 < width <= MAX_SIDE and 0 < height <= MAX_SIDE) or stamp > INT64_MAX:
        return False
    if depth and not 0 < int(meta[5]) <= 1_000_000_000:
        return False
    slot = width * height * (5 if depth else 3) + SEQUENCE_BYTES
    return offset % slot == 0 and offset // slot < MAX_SLOTS


class ShmFrameReader:
    """Read-only reader of one craft_sensors shared-memory camera."""

    def __init__(self, name: str, depth: bool, shm_dir: str = '/dev/shm'):
        if len(name) < 2 or name[0] != '/' or '/' in name[1:]:
            raise ValueError('shared memory name must be /name')
        self.path = shm_dir.rstrip('/') + name
        self.depth = depth
        self._fd = None
        self._map = None
        self._ident = None

    def close(self):
        if self._map is not None:
            self._map.close()
        if self._fd is not None:
            os.close(self._fd)
        self._map = None
        self._fd = None
        self._ident = None

    def snapshot(self, meta) -> Frame | None:
        """Copy the color image the metadata points at, or None if it is gone or busy."""
        meta = [int(v) for v in meta]
        if not valid_meta(meta, self.depth):
            return None
        try:
            current = os.stat(self.path)
        except OSError:
            self.close()
            return None
        # Producer unlink/recreate shows up as a new inode.
        if self._fd is not None and self._ident != (current.st_dev, current.st_ino):
            self.close()
        if self._fd is None:
            try:
                self._fd = os.open(self.path, os.O_RDONLY)
            except OSError:
                return None
        try:
            fcntl.flock(self._fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except OSError:
            return None  # producer is writing; the next frame_meta will do
        try:
            st = os.fstat(self._fd)
            self._ident = (st.st_dev, st.st_ino)
            offset, width, height, sequence, stamp_ns = meta[:5]
            pixels = width * height
            payload = pixels * (5 if self.depth else 3)
            if st.st_size < offset + payload + SEQUENCE_BYTES:
                return None
            if self._map is None or len(self._map) != st.st_size:
                if self._map is not None:
                    self._map.close()
                self._map = mmap.mmap(self._fd, st.st_size, prot=mmap.PROT_READ)
            trailer = self._map[offset + payload:offset + payload + SEQUENCE_BYTES]
            if int.from_bytes(trailer, sys.byteorder) != sequence:
                return None  # slot already overwritten by a newer frame
            color = self._map[offset:offset + pixels * 3]  # bytes copy, no buffer export kept
        finally:
            fcntl.flock(self._fd, fcntl.LOCK_UN)
        bgr = np.frombuffer(color, dtype=np.uint8).reshape(height, width, 3)
        return Frame(bgr=bgr, sequence=sequence, stamp_ns=stamp_ns)


_CHANNELS = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1, '8uc1': 1, '8uc3': 3}


def image_to_bgr(encoding: str, height: int, width: int, step: int, data) -> np.ndarray:
    """sensor_msgs/Image fields -> HxWx3 BGR uint8 (8-bit encodings only)."""
    encoding = encoding.lower()
    if encoding not in _CHANNELS:
        raise ValueError(f'unsupported image encoding {encoding!r}')
    channels = _CHANNELS[encoding]
    row = width * channels
    try:
        buffer = np.frombuffer(memoryview(data), dtype=np.uint8)
    except TypeError:
        buffer = np.asarray(data, dtype=np.uint8)
    if width <= 0 or height <= 0 or step < row or buffer.size < step * height:
        raise ValueError('image size does not match its data')
    image = buffer[:step * height].reshape(height, step)[:, :row].reshape(height, width, channels)
    if encoding == 'rgb8':
        image = image[..., ::-1]
    elif encoding == 'rgba8':
        image = image[..., 2::-1]
    elif encoding == 'bgra8':
        image = image[..., :3]
    elif channels == 1:
        image = np.repeat(image, 3, axis=2)
    return np.ascontiguousarray(image)


def compressed_to_bgr(data) -> np.ndarray:
    """sensor_msgs/CompressedImage data (JPEG/PNG) -> BGR."""
    import cv2
    image = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('compressed image could not be decoded')
    return image


def encode_jpeg(bgr: np.ndarray, max_side: int = 512, quality: int = 80) -> bytes:
    """Shrink so the longer side is at most max_side (never enlarge) and JPEG-encode."""
    import cv2
    height, width = bgr.shape[:2]
    scale = max_side / max(height, width)
    if max_side > 0 and scale < 1.0:
        size = (max(1, round(width * scale)), max(1, round(height * scale)))
        bgr = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return encoded.tobytes()
