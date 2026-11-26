import unittest

from openpilot.sunnypilot.webhud.replay_video import (MAX_BURST, Frame, frame_payload, frame_table, frames_due,
                                                       split_access_units, transcode_cmd, STREAM_GOP)
from openpilot.sunnypilot.webhud.video import FRAME_HEADER


def nal(t: int, body: bytes = b"\xaa\xbb", long_code: bool = True) -> bytes:
  return (b"\x00\x00\x00\x01" if long_code else b"\x00\x00\x01") + bytes([t]) + body


AUD, SPS, PPS, IDR, SLICE = 9, 7, 8, 5, 1


def stream(n: int, gop: int = 5) -> bytes:
  out = b""
  for i in range(n):
    out += nal(AUD, b"\x10")
    out += nal(SPS) + nal(PPS) + nal(IDR, bytes([i])) if i % gop == 0 else nal(SLICE, bytes([i]), long_code=False)
  return out


class TestAccessUnits(unittest.TestCase):
  def test_split_at_delimiters_with_keyframes(self):
    data = stream(12)
    units = split_access_units(data)
    self.assertEqual(len(units), 12)
    self.assertEqual([u[2] for u in units], [i % 5 == 0 for i in range(12)])
    # units tile the stream and each starts with its delimiter
    self.assertEqual(units[0][0], 0)
    for (o1, l1, _), (o2, _, _) in zip(units, units[1:], strict=False):
      self.assertEqual(o1 + l1, o2)
    self.assertEqual(units[-1][0] + units[-1][1], len(data))
    off, ln, _ = units[7]
    self.assertTrue(data[off:off + ln].startswith(b"\x00\x00\x00\x01\x09"))
    self.assertIn(bytes([7]), data[off:off + ln][-1:])   # unit 7 carries its own slice

  def test_three_byte_start_codes_and_no_delimiters(self):
    data = nal(AUD, long_code=False) + nal(IDR, long_code=False)
    self.assertEqual(split_access_units(data), [(0, len(data), True)])
    self.assertEqual(split_access_units(nal(SLICE)), [])


class TestFrames(unittest.TestCase):
  def test_table_sorted_by_time_one_per_file_frame(self):
    entries = [(10.10, 102, 9_000, 2), (10.00, 100, 8_000, 0), (10.05, 101, 8_500, 1), (10.051, 101, 8_500, 1)]
    t = frame_table(entries)
    self.assertEqual([f.seg_id for f in t], [0, 1, 2])
    self.assertEqual([f.frame_id for f in t], [100, 101, 102])

  def test_payload_header_matches_live(self):
    p = frame_payload("wide", Frame(1.0, 7202, 8329938678525, 0), True, b"\x00\x00\x00\x01\x09")
    kind, flags, cam, fid, eof = FRAME_HEADER.unpack(p[:FRAME_HEADER.size])
    self.assertEqual((kind, flags, cam, fid, eof), (1, 1, 1, 7202, 8329938678525))
    self.assertEqual(p[FRAME_HEADER.size:], b"\x00\x00\x00\x01\x09")
    _, flags, cam, _, _ = FRAME_HEADER.unpack(frame_payload("road", Frame(0, 1, 2, 3), False, b"")[:FRAME_HEADER.size])
    self.assertEqual((flags, cam), (0, 0))


class TestDue(unittest.TestCase):
  times = [i * 0.05 for i in range(40)]           # 20 fps
  keys = [i % 5 == 0 for i in range(40)]

  def test_first_call_starts_at_the_keyframe_before_now(self):
    idx, restart = frames_due(self.times, self.keys, None, 0.37)   # frame 7 is current
    self.assertTrue(restart)
    self.assertEqual(idx, [5, 6, 7])

  def test_playing_sends_only_new_frames(self):
    idx, restart = frames_due(self.times, self.keys, 0.35, 0.46)
    self.assertFalse(restart)
    self.assertEqual(idx, [8, 9])

  def test_paused_sends_nothing(self):
    self.assertEqual(frames_due(self.times, self.keys, 0.35, 0.35)[0], [])

  def test_seek_back_and_jump_ahead_restart_at_keyframes(self):
    idx, restart = frames_due(self.times, self.keys, 1.5, 0.62)    # back: frame 12, keyframe 10
    self.assertTrue(restart)
    self.assertEqual(idx, [10, 11, 12])
    idx, restart = frames_due(self.times, self.keys, 0.1, 1.83)    # +1.7 s: frame 36, keyframe 35
    self.assertTrue(restart)
    self.assertEqual(idx, [35, 36])

  def test_fast_play_burst_is_capped_to_the_newest_keyframe_run(self):
    idx, restart = frames_due(self.times, self.keys, 0.0, 0.92)    # 18 new frames in one step
    self.assertLessEqual(len(idx), MAX_BURST)
    self.assertTrue(self.keys[idx[0]])
    self.assertEqual(idx[-1], 18)
    self.assertTrue(restart)

  def test_before_the_first_frame(self):
    self.assertEqual(frames_due(self.times, self.keys, None, -1.0)[0], [])


class TestTranscode(unittest.TestCase):
  def test_matches_the_livestream_encoder(self):
    cmd = " ".join(transcode_cmd("in.hevc", "out.h264"))
    for part in ("scale=1280:720", "-g 5", "-keyint_min 5", "-bf 0", "repeat-headers=1:aud=1", "-f h264", "-b:v 5M"):
      self.assertIn(part, cmd)
    self.assertEqual(STREAM_GOP, 5)


if __name__ == "__main__":
  unittest.main()
