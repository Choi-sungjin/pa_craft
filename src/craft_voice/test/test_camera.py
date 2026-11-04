"""camera.CameraFrames: GPT sees the camera frame the planning state machine's vision input came from."""
import os
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from craft_voice.camera import CameraFrames, stamp_ns
from craft_voice.frames import ShmFrameReader

W, H = 8, 6
SLOT = W * H * 5 + 8  # D435 slot: BGR + Z16 depth + uint64 sequence


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def stamp(seq):
    return 1_700_000_000_000_000_000 + seq * 33_333_333


def put(path, seq):
    """Like craft_sensors: frame `seq` (all pixels = seq % 256) goes into slot seq % 4; returns its frame_meta."""
    slot = seq % 4
    if not os.path.exists(path):
        with open(path, 'wb') as out:
            out.truncate(SLOT * 4)
    with open(path, 'r+b') as out:
        out.seek(slot * SLOT)
        out.write(np.full((H, W, 3), seq % 256, np.uint8).tobytes())
        out.write(np.zeros((H, W), np.uint16).tobytes())
        out.write(seq.to_bytes(8, sys.byteorder))
    return [slot * SLOT, W, H, seq, stamp(seq), 1_000_000]


def setup(tmp_path, **kwargs):
    clock = Clock()
    frames = CameraFrames(ShmFrameReader('/cam', depth=True, shm_dir=str(tmp_path)), clock=clock, **kwargs)
    return frames, clock, str(tmp_path) + '/cam'


def test_input_frame_is_the_one_perception_used_not_the_newest(tmp_path):
    frames, clock, path = setup(tmp_path)
    for seq in range(1, 6):
        frames.on_meta(put(path, seq))
    assert frames.on_input(stamp(3), '/pucks')               # puck observations computed from frame 3
    for seq in range(6, 9):
        frames.on_meta(put(path, seq))                       # frame 7 reuses the slot of frame 3
    clock.t += 0.05
    picture = frames.pick()
    assert picture.source == 'input' and picture.stamp_ns == stamp(3) and picture.topic == '/pucks'
    assert np.all(picture.bgr == 3)                          # copied in time, kept after the slot was reused


def test_slot_already_reused_falls_back_to_the_newest_frame(tmp_path):
    frames, clock, path = setup(tmp_path)
    for seq in range(1, 8):
        frames.on_meta(put(path, seq))                       # slot of frame 2 now holds frame 6
    assert not frames.on_input(stamp(2), '/pucks')
    assert frames.stats == {'input_frames': 0, 'input_unmatched': 0, 'input_lost': 1}
    picture = frames.pick()
    assert picture.source == 'latest' and np.all(picture.bgr == 7) and picture.stamp_ns == stamp(7)


def test_stamp_of_no_known_frame_falls_back(tmp_path):
    frames, clock, path = setup(tmp_path)
    frames.on_meta(put(path, 1))
    assert not frames.on_input(stamp(99), '/objects') and not frames.on_input(0, '/objects')
    assert frames.stats['input_unmatched'] == 1
    assert frames.pick().source == 'latest'


def test_state_change_gets_the_input_frame_it_had_at_that_moment(tmp_path):
    frames, clock, path = setup(tmp_path)
    for seq, t in ((1, 100.0), (2, 100.5), (3, 101.0)):
        clock.t = t
        frames.on_meta(put(path, seq))
        assert frames.on_input(stamp(seq), '/objects')
    clock.t = 101.2
    assert np.all(frames.pick(at=100.7).bgr == 2)            # state changed at 100.7: the input it had then
    assert np.all(frames.pick(at=100.45).bgr == 2)           # arrived 0.05 s after the change: still that moment
    assert np.all(frames.pick(at=100.3).bgr == 1)
    assert np.all(frames.pick(at=99.0).bgr == 1)             # before every kept input: the closest one
    assert np.all(frames.pick().bgr == 3)                    # now: the newest input


def test_old_inputs_and_frames_expire(tmp_path):
    frames, clock, path = setup(tmp_path, max_age_s=2.0)
    frames.on_meta(put(path, 1))
    frames.on_input(stamp(1), '/objects')
    clock.t = 102.01
    assert frames.pick(at=100.0) is None                     # input and camera both older than 2 s
    frames.on_meta(put(path, 2))
    picture = frames.pick(at=100.0)
    assert picture.source == 'latest' and np.all(picture.bgr == 2)
    assert frames.ages() == (0.0, pytest.approx(2.01))


def test_one_copy_per_frame_and_at_most_every_interval(tmp_path):
    frames, clock, path = setup(tmp_path, min_interval_s=0.2)
    frames.on_meta(put(path, 1))
    assert frames.on_input(stamp(1), '/objects')
    assert not frames.on_input(stamp(1), '/pucks')           # same frame from the other topic
    clock.t += 0.1
    frames.on_meta(put(path, 2))
    assert not frames.on_input(stamp(2), '/objects')         # 0.1 s after the last copy
    clock.t += 0.15
    frames.on_meta(put(path, 3))
    assert frames.on_input(stamp(3), '/objects')
    assert frames.stats['input_frames'] == 2


def test_latest_mode_ignores_inputs(tmp_path):
    frames, clock, path = setup(tmp_path, mode='latest')
    frames.on_meta(put(path, 1))
    frames.on_meta(put(path, 2))
    assert not frames.on_input(stamp(1), '/objects')
    picture = frames.pick(at=99.0)
    assert picture.source == 'latest' and np.all(picture.bgr == 2)
    with pytest.raises(ValueError):
        CameraFrames(mode='newest')


def test_image_topic_camera_matches_by_header_stamp():
    clock = Clock()
    frames = CameraFrames(None, clock=clock)

    def image(value):
        return SimpleNamespace(encoding='bgr8', height=H, width=W, step=W * 3, data=bytes([value]) * (H * W * 3))
    frames.on_image(image(10), stamp(1))
    frames.on_image(image(20), stamp(2))
    assert frames.on_input(stamp(1), '/objects')
    picture = frames.pick()
    assert picture.source == 'input' and np.all(picture.bgr == 10)
    frames.on_image(SimpleNamespace(encoding='16uc1', height=H, width=W, step=W * 2, data=b''), stamp(3))
    clock.t += 3.0
    assert frames.pick() is None                             # unreadable newest image: nothing, no exception


def test_stamp_ns():
    assert stamp_ns(SimpleNamespace(sec=2, nanosec=5)) == 2_000_000_005 and stamp_ns(None) == 0


def test_callbacks_and_pick_from_different_threads(tmp_path):
    frames = CameraFrames(ShmFrameReader('/cam', depth=True, shm_dir=str(tmp_path)), min_interval_s=0.0)
    path = str(tmp_path) + '/cam'
    stop, errors, produced = threading.Event(), [], []

    def produce():
        seq = 1
        try:
            while not stop.is_set():
                frames.on_meta(put(path, seq))
                frames.on_input(stamp(seq), '/objects')
                seq += 1
        except Exception as error:  # pragma: no cover - reported below
            errors.append(error)
        produced.append(seq)

    def consume():
        try:
            while not stop.is_set():
                picture = frames.pick(at=time.monotonic() - 0.01)
                assert picture is None or picture.bgr.shape == (H, W, 3)
                time.sleep(0)  # let the producer run too
        except Exception as error:  # pragma: no cover - reported below
            errors.append(error)
    threads = [threading.Thread(target=produce), threading.Thread(target=consume)]
    for thread in threads:
        thread.start()
    time.sleep(0.4)
    stop.set()
    for thread in threads:
        thread.join(2.0)
    frames.close()
    assert errors == [] and frames.stats['input_frames'] > 0
    assert produced and produced[0] > 256  # the producer ran past where uint8 pixel values wrap


def test_same_frame_from_two_topics_is_copied_once(tmp_path):
    frames, clock, path = setup(tmp_path, min_interval_s=0.0)
    frames.on_meta(put(path, 1))
    assert frames.on_input(stamp(1), '/objects')
    clock.t += 0.5
    assert not frames.on_input(stamp(1), '/pucks')           # puck observations of the same frame
    assert frames.stats['input_frames'] == 1 and frames.pick().topic == '/objects'


def test_a_busy_slot_is_not_lost_for_the_other_topic(tmp_path):
    import fcntl
    frames, clock, path = setup(tmp_path)
    frames.on_meta(put(path, 1))
    with open(path, 'rb') as writer:
        fcntl.flock(writer, fcntl.LOCK_EX)                   # craft_sensors is writing another slot
        assert not frames.on_input(stamp(1), '/objects')
        fcntl.flock(writer, fcntl.LOCK_UN)
    assert frames.on_input(stamp(1), '/pucks')               # same frame, other topic: still copied
    assert frames.pick().topic == '/pucks' and frames.stats['input_lost'] == 1


def test_kept_for_max_age_whatever_the_interval():
    clock = Clock(0.0)
    frames = CameraFrames(None, min_interval_s=0.09, max_age_s=2.0, clock=clock)
    for i in range(30):                                      # one input frame every 0.1 s for 3 s
        clock.t = i * 0.1
        frames.on_image(SimpleNamespace(encoding='bgr8', height=H, width=W, step=W * 3, data=bytes([i]) * (H * W * 3)),
                        i + 1)
        assert frames.on_input(i + 1, '/objects')
    clock.t = 3.0
    assert np.all(frames.pick(at=1.15).bgr == 12)            # 1.85 s old, still the frame of that moment


def test_unreadable_image_never_raises():
    frames = CameraFrames(None)
    frames.on_image(SimpleNamespace(format='jpeg', data=b''), stamp(1))   # cv2.error, not ValueError
    assert not frames.on_input(stamp(1), '/objects') and frames.pick() is None
