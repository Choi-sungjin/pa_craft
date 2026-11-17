import array
import fcntl
import os
import sys

import cv2
import numpy as np
import pytest

from craft_voice.frames import (ShmFrameReader, compressed_to_bgr, encode_jpeg, image_to_bgr,
                                valid_meta)

W, H = 8, 6


def write_shm(path, depth, slot, sequence, color, slots=4, w=W, h=H):
    slot_size = w * h * (5 if depth else 3) + 8
    if not os.path.exists(path):
        with open(path, 'wb') as out:
            out.truncate(slot_size * slots)
    with open(path, 'r+b') as out:
        out.seek(slot * slot_size)
        out.write(color.tobytes())
        if depth:
            out.write(np.full((h, w), 1234, np.uint16).tobytes())
        out.write(sequence.to_bytes(8, sys.byteorder))
    return slot_size


def pattern(seed):
    return (np.arange(H * W * 3, dtype=np.uint16).reshape(H, W, 3) * 7 + seed).astype(np.uint8)


@pytest.fixture
def shm(tmp_path):
    return str(tmp_path)


def test_d435_slot_is_copied(shm):
    color = pattern(3)
    slot_size = write_shm(shm + '/cam', True, 2, 7, color)
    reader = ShmFrameReader('/cam', depth=True, shm_dir=shm)
    frame = reader.snapshot([2 * slot_size, W, H, 7, 123, 1_000_000])
    assert frame is not None and frame.sequence == 7 and frame.stamp_ns == 123
    assert frame.bgr.shape == (H, W, 3) and np.array_equal(frame.bgr, color)


def test_c310_slot_is_copied(shm):
    color = pattern(9)
    slot_size = write_shm(shm + '/c310', False, 3, 41, color)
    reader = ShmFrameReader('/c310', depth=False, shm_dir=shm)
    frame = reader.snapshot([3 * slot_size, W, H, 41, 5])
    assert np.array_equal(frame.bgr, color)


def test_overwritten_slot_is_rejected(shm):
    slot_size = write_shm(shm + '/cam', True, 1, 8, pattern(1))
    reader = ShmFrameReader('/cam', depth=True, shm_dir=shm)
    assert reader.snapshot([slot_size, W, H, 7, 0, 1_000_000]) is None


@pytest.mark.parametrize('meta, depth', [
    ([0, W, H, 1, 0], True),                        # D435 needs 6 values
    ([0, W, H, 1, 0, 1_000_000], False),            # C310 has 5
    ([0, 0, H, 1, 0], False),                       # zero width
    ([1, W, H, 1, 0], False),                       # offset not on a slot
    ([4 * (W * H * 3 + 8), W, H, 1, 0], False),     # slot index 4
    ([0, W, H, 1, 0, 0], True),                     # depth unit 0
    ([0, W, H, 1, 2**63], False),                   # stamp beyond int64
])
def test_invalid_metadata(meta, depth):
    assert not valid_meta(meta, depth)


def test_missing_file_and_short_file(shm):
    reader = ShmFrameReader('/nothing', depth=False, shm_dir=shm)
    assert reader.snapshot([0, W, H, 1, 0]) is None
    with open(shm + '/short', 'wb') as out:
        out.write(b'\0' * 10)
    assert ShmFrameReader('/short', depth=False, shm_dir=shm).snapshot([0, W, H, 1, 0]) is None


def test_busy_producer_then_free(shm):
    write_shm(shm + '/cam', False, 0, 5, pattern(2))
    reader = ShmFrameReader('/cam', depth=False, shm_dir=shm)
    producer = os.open(shm + '/cam', os.O_RDWR)
    try:
        fcntl.flock(producer, fcntl.LOCK_EX)
        assert reader.snapshot([0, W, H, 5, 0]) is None
        fcntl.flock(producer, fcntl.LOCK_UN)
        assert reader.snapshot([0, W, H, 5, 0]) is not None
    finally:
        os.close(producer)


def test_recreated_file_is_reopened(shm):
    write_shm(shm + '/cam', False, 0, 5, pattern(2))
    reader = ShmFrameReader('/cam', depth=False, shm_dir=shm)
    assert np.array_equal(reader.snapshot([0, W, H, 5, 0]).bgr, pattern(2))
    os.unlink(shm + '/cam')
    write_shm(shm + '/cam', False, 0, 5, pattern(50))
    assert np.array_equal(reader.snapshot([0, W, H, 5, 0]).bgr, pattern(50))
    reader.close()


def test_bad_name():
    with pytest.raises(ValueError):
        ShmFrameReader('craft_vision', depth=True)


def test_image_conversions():
    rgb = pattern(4)
    out = image_to_bgr('rgb8', H, W, W * 3, rgb.tobytes())
    assert np.array_equal(out, rgb[..., ::-1])
    padded = np.zeros((H, W * 3 + 4), np.uint8)
    padded[:, :W * 3] = rgb.reshape(H, W * 3)
    assert np.array_equal(image_to_bgr('bgr8', H, W, W * 3 + 4, array.array('B', padded.tobytes())), rgb)
    mono = np.arange(H * W, dtype=np.uint8).reshape(H, W)
    assert np.array_equal(image_to_bgr('mono8', H, W, W, mono.tobytes()), np.dstack([mono] * 3))
    bgra = np.dstack([rgb, np.full((H, W), 255, np.uint8)])
    assert np.array_equal(image_to_bgr('bgra8', H, W, W * 4, bgra.tobytes()), rgb)
    assert np.array_equal(image_to_bgr('rgba8', H, W, W * 4, bgra.tobytes()), rgb[..., ::-1])
    with pytest.raises(ValueError):
        image_to_bgr('16UC1', H, W, W * 2, b'\0' * (H * W * 2))
    with pytest.raises(ValueError):
        image_to_bgr('bgr8', H, W, W * 3, b'\0' * 10)


def test_jpeg_resize_and_compressed_round_trip():
    big = np.random.default_rng(0).integers(0, 255, (360, 640, 3), dtype=np.uint8)
    jpeg = encode_jpeg(big, max_side=512, quality=80)
    assert jpeg[:2] == b'\xff\xd8'
    assert cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape == (288, 512, 3)
    small = encode_jpeg(big[:100, :200], max_side=512)
    assert compressed_to_bgr(small).shape == (100, 200, 3)  # never enlarged
    with pytest.raises(ValueError):
        compressed_to_bgr(b'not an image')
