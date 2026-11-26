"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The camera view (`/video`) during a replay: the segment's recorded cameras played in step with the log, in exactly
the live stream's format (video.py), so the page's camera window, camera-on-road and NPU detector can be developed
and checked against real pixels on a PC.

The recorded files are HEVC at the sensor's size (fcamera.hevc: road, ecamera.hevc: wide). Each is transcoded once,
with ffmpeg, to what the livestream encoder sends (loggerd.h StreamEncoderSettings: H.264 at 1280x720, 5 Mbit/s, a
keyframe every 5 frames), kept next to it as `<name>.stream.h264` and split into access units by its access unit
delimiters. Frame n of the file is the encode index entry with segmentId n (narrowRoadEncodeIdx /
wideRoadEncodeIdx), which gives its frameId and timestampEof, so frames, model and calibration line up by frame id as
they do live. Playback follows the replay's clock: pause, speed and seeks (a seek restarts at the keyframe before the
new position).
"""
import json
import os
import subprocess
import threading
import time
from bisect import bisect_right
from dataclasses import dataclass, field

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.webhud.video import (CAMERAS, FLAG_KEY, FRAME_HEADER, HEIGHT_INIT, KIND_FRAME, VideoViewer,
                                                camera_transforms, model_overlay)

STREAM_W, STREAM_H = 1280, 720     # loggerd.h livestream_width/height on a comma four (mici)
STREAM_GOP = 5                     # StreamEncoderSettings gop_size
STREAM_BITRATE = "5M"
FILES = {"road": "fcamera.hevc", "wide": "ecamera.hevc"}
INDEX_SERVICES = {"road": "narrowRoadEncodeIdx", "wide": "wideRoadEncodeIdx"}
SIDE_SERVICES = ("modelV2", "radarState", "selfdriveState", "longitudinalPlan", "extrinsicsCalibration",
                 "deviceState", "narrowRoadCameraState", "carParams")
JUMP_S = 1.0          # the replay clock moving more than this (or backwards) is a seek: restart at a keyframe
MAX_BURST = 5         # frames sent in one step at most, one GOP: a viewer drops its backlog past 6 (video.py MAX_QUEUED_FRAMES)
STEP_S = 0.01
AUD = 9               # H.264 access unit delimiter NAL type
IDR = 5


# ---- pure helpers (tested) -----------------------------------------------------------------------

def nal_starts(data: bytes) -> list[tuple[int, int]]:
  """(offset of the start code, NAL type) for each NAL unit of an Annex B stream."""
  out = []
  i, n = 0, len(data)
  while True:
    i = data.find(b"\x00\x00\x01", i)
    if i < 0 or i + 3 >= n:
      return out
    start = i - 1 if i > 0 and data[i - 1] == 0 else i
    out.append((start, data[i + 3] & 0x1F))
    i += 3


def split_access_units(data: bytes) -> list[tuple[int, int, bool]]:
  """(offset, length, keyframe) of each access unit, an access unit starting at each delimiter (ffmpeg's aud=1)."""
  nals = nal_starts(data)
  bounds = [off for off, t in nals if t == AUD]
  out = []
  for k, off in enumerate(bounds):
    end = bounds[k + 1] if k + 1 < len(bounds) else len(data)
    key = any(t == IDR and off <= o < end for o, t in nals)
    out.append((off, end - off, key))
  return out


@dataclass
class Frame:
  t: float          # log time (s, the replay's clock)
  frame_id: int
  ts_eof: int       # ns
  seg_id: int       # index in the recorded file


def frame_table(entries: list[tuple[float, int, int, int]]) -> list[Frame]:
  """Encode index entries (log time, frameId, timestampEof, segmentId) as frames sorted by time, one per segmentId."""
  seen: dict[int, Frame] = {}
  for t, fid, eof, sid in entries:
    if sid not in seen:
      seen[sid] = Frame(t, fid, eof, sid)
  return sorted(seen.values(), key=lambda f: f.t)


def frames_due(times: list[float], keys: list[bool], sent_t: float | None, now: float) -> tuple[list[int], bool]:
  """Which frames to send now: indices into `times` (sorted log times) with keyframe flags `keys`, and whether this
  was a restart. Normally every frame after `sent_t` up to `now`; after a seek (no `sent_t`, the clock went back, or
  it jumped more than JUMP_S) from the keyframe at or before `now`; at most MAX_BURST (a burst keeps its newest
  keyframe run)."""
  last = bisect_right(times, now) - 1
  if last < 0:
    return [], sent_t is None
  restart = sent_t is None or now < sent_t - 1e-6 or now - sent_t > JUMP_S
  if restart:
    first = last
    while first > 0 and not keys[first]:
      first -= 1
  else:
    first = bisect_right(times, sent_t + 1e-6)
  if first > last:
    return [], restart
  idx = list(range(first, last + 1))
  if len(idx) > MAX_BURST:
    k = max((i for i in idx if keys[i]), default=None)
    idx = idx[idx.index(k):] if k is not None and last - k < MAX_BURST else idx[-MAX_BURST:]
    restart = True
  return idx, restart


def frame_payload(camera: str, frame: Frame, key: bool, unit: bytes) -> bytes:
  return FRAME_HEADER.pack(KIND_FRAME, FLAG_KEY if key else 0, CAMERAS.index(camera), frame.frame_id, frame.ts_eof) + unit


def transcode_cmd(src: str, dst: str) -> list[str]:
  return ["ffmpeg", "-loglevel", "error", "-y", "-i", src, "-fps_mode", "passthrough",
          "-vf", f"scale={STREAM_W}:{STREAM_H}", "-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency",
          "-profile:v", "main", "-b:v", STREAM_BITRATE, "-maxrate", STREAM_BITRATE, "-bufsize", "2M",
          "-g", str(STREAM_GOP), "-keyint_min", str(STREAM_GOP), "-sc_threshold", "0", "-bf", "0",
          "-x264-params", "repeat-headers=1:aud=1", "-f", "h264", dst]


# ---- one segment's recorded cameras ---------------------------------------------------------------

@dataclass
class CameraTrack:
  frames: list[Frame]
  units: list[tuple[int, int, bool]]
  data: bytes
  times: list[float] = field(default_factory=list)
  keys: list[bool] = field(default_factory=list)

  def __post_init__(self):
    n = min(len(self.frames), len(self.units))
    self.frames, self.units = self.frames[:n], self.units[:n]
    self.times = [f.t for f in self.frames]
    self.keys = [u[2] for u in self.units]

  def payload(self, camera: str, i: int) -> bytes:
    off, ln, key = self.units[i]
    return frame_payload(camera, self.frames[i], key, self.data[off:off + ln])


class SegmentVideo:
  """The recorded cameras of one segment, transcoded, with the model and calibration messages on the same clock."""

  def __init__(self, seg_dir: str, rlog: str):
    self.dir = seg_dir
    self.tracks: dict[str, CameraTrack] = {}
    self.models: list[tuple[float, str]] = []
    self.calibs: list[tuple[float, str]] = []
    self._load(rlog)

  @staticmethod
  def present(seg_dir: str) -> bool:
    return any(os.path.isfile(os.path.join(seg_dir, f)) for f in FILES.values())

  def _stream(self, cam: str) -> bytes | None:
    src = os.path.join(self.dir, FILES[cam])
    if not os.path.isfile(src):
      return None
    dst = src[:-len(".hevc")] + ".stream.h264"
    if not os.path.isfile(dst) or os.path.getmtime(dst) < os.path.getmtime(src):
      t0 = time.monotonic()
      tmp = dst + ".tmp"
      subprocess.run(transcode_cmd(src, tmp), check=True)
      os.replace(tmp, dst)
      cloudlog.info(f"webhud replay video: transcoded {src} in {time.monotonic() - t0:.0f}s")
    with open(dst, "rb") as f:
      return f.read()

  def _load(self, rlog: str) -> None:
    from openpilot.tools.lib.logreader import LogReader
    entries: dict[str, list] = {c: [] for c in CAMERAS}
    side: list = []
    for evt in LogReader(rlog):
      which = evt.which()
      t = evt.logMonoTime * 1e-9
      for cam, svc in INDEX_SERVICES.items():
        if which == svc:
          e = getattr(evt, svc)
          entries[cam].append((t, e.frameId, e.timestampEof, e.segmentId))
      if which in SIDE_SERVICES:
        side.append((t, which, evt))
    for cam in CAMERAS:
      data = self._stream(cam)
      if data and entries[cam]:
        self.tracks[cam] = CameraTrack(frame_table(entries[cam]), split_access_units(data), data)
    self._side(side)

  def _side(self, side: list) -> None:
    latest: dict = {}
    device_type, sensor = "mici", "os04c10"
    long_control = False
    sizes = dict.fromkeys(self.tracks, (STREAM_W, STREAM_H))
    last_calib = None
    for t, which, msg in side:
      body = getattr(msg, which)
      latest[which] = body
      if which == "deviceState":
        device_type = str(body.deviceType)
      elif which == "narrowRoadCameraState":
        sensor = str(body.sensor)
      elif which == "carParams":
        long_control = bool(body.openpilotLongitudinalControl)
      elif which == "extrinsicsCalibration" and sizes:
        ok = body.calStatus == 1 and len(body.rpyCalib) == 3
        text = json.dumps(camera_transforms(device_type, sensor, sizes, list(body.rpyCalib) if ok else None,
                                            list(body.wideFromDeviceEuler) if ok else None), separators=(",", ":"))
        if text != last_calib:
          self.calibs.append((t, text))
          last_calib = text
      elif which == "modelV2":
        cal = latest.get("extrinsicsCalibration")
        path_z = cal.height[0] if cal is not None and len(cal.height) else HEIGHT_INIT
        msg_ = model_overlay(body, latest.get("radarState"), latest.get("selfdriveState"), latest.get("longitudinalPlan"),
                             long_control, path_z, 0.0)
        self.models.append((t, json.dumps(msg_, separators=(",", ":"))))


# ---- the hub /video uses during a replay ----------------------------------------------------------

class ReplayVideoHub:
  """VideoHub's interface (add/remove/stop), fed from the replay's recorded cameras instead of the encoder."""

  def __init__(self, engine):
    self.engine = engine
    self.viewers: set[VideoViewer] = set()
    self.lock = threading.Lock()
    self.stop_event = threading.Event()
    self._thread: threading.Thread | None = None
    self._segs: dict[str, SegmentVideo | None] = {}   # dir -> loaded (None while loading or failed)
    self._loading: set[str] = set()
    self.error: str | None = None
    self._sent: dict[str, float | None] = {}
    self._model_t: float | None = None
    self._calib: str | None = None

  def _source(self):
    eng = self.engine
    src = getattr(eng, "replay", None)
    if getattr(eng, "mode", None) != "replay" or src is None or not hasattr(src, "segments") or src.cur is None:
      return None
    return src

  def _seg_dir(self, src) -> tuple[str, str] | None:
    path = src.segments.get(src.cur)
    if not path:
      return None
    return os.path.dirname(path), path

  def available(self) -> bool:
    """A route is replaying and some segment of it has recorded cameras next to its log. Asked when a viewer
    connects, which may be before the replay has loaded its first segment: the viewer then waits here for it."""
    eng = self.engine
    src = getattr(eng, "replay", None)
    if getattr(eng, "mode", None) != "replay" or src is None or not hasattr(src, "segments"):
      return False
    return any(SegmentVideo.present(os.path.dirname(p)) for p in src.segments.values())

  def add(self, viewer: VideoViewer) -> None:
    viewer.push_text(json.dumps({"type": "hello", "cameras": list(CAMERAS), "replay": True}))
    if self._calib:
      viewer.push_text(self._calib)
    with self.lock:
      self.viewers.add(viewer)
      self._sent = {}   # newcomers start at a keyframe
      if self._thread is None or not self._thread.is_alive():
        self._thread = threading.Thread(target=self.run, name="webhud-replay-video", daemon=True)
        self._thread.start()

  def remove(self, viewer: VideoViewer) -> None:
    with self.lock:
      self.viewers.discard(viewer)

  def stop(self) -> None:
    self.stop_event.set()
    with self.lock:
      viewers = list(self.viewers)
    for v in viewers:
      v.close()

  def _alive(self) -> list[VideoViewer]:
    with self.lock:
      return [v for v in self.viewers if v.alive]

  def _broadcast(self, text: str) -> None:
    for v in self._alive():
      v.push_text(text)

  def _get(self, seg_dir: str, rlog: str) -> SegmentVideo | None:
    if seg_dir in self._segs:
      return self._segs[seg_dir]
    if seg_dir not in self._loading:
      self._loading.add(seg_dir)

      def load():
        try:
          self._segs[seg_dir] = SegmentVideo(seg_dir, rlog)
          self.error = None
        except Exception as e:
          cloudlog.exception("webhud replay video: load failed")
          self.error = f"replay video: {type(e).__name__}: {e}"
          self._segs[seg_dir] = None
          self._broadcast(json.dumps({"type": "error", "message": self.error}))
        finally:
          self._loading.discard(seg_dir)
      threading.Thread(target=load, name="webhud-replay-video-load", daemon=True).start()
    return None

  def run(self) -> None:
    while not self.stop_event.is_set():
      viewers = self._alive()
      if not viewers:
        with self.lock:
          if not self.viewers:
            self._thread = None
            return
      try:
        self._step(viewers)
      except Exception:
        cloudlog.exception("webhud replay video: step")
        time.sleep(0.5)
      time.sleep(STEP_S)

  def _step(self, viewers: list[VideoViewer]) -> None:
    src = self._source()
    if src is None:
      return
    loc = self._seg_dir(src)
    if loc is None:
      return
    seg = self._get(*loc)
    if seg is None:   # loading (the first time, the transcode takes about 10 s a camera) or failed (self.error)
      return
    now = src.now
    # calibration and model, as live: the newest at or before now
    i = bisect_right([t for t, _ in seg.calibs], now) - 1
    if i >= 0 and seg.calibs[i][1] != self._calib:
      self._calib = seg.calibs[i][1]
      self._broadcast(self._calib)
    mt = [t for t, _ in seg.models]
    j = bisect_right(mt, now) - 1
    if j >= 0 and (self._model_t is None or mt[j] != self._model_t):
      self._model_t = mt[j]
      self._broadcast(seg.models[j][1])
    wanted = {v.camera for v in viewers}
    for cam in wanted:
      track = seg.tracks.get(cam)
      if track is None:
        continue
      idx, _ = frames_due(track.times, track.keys, self._sent.get(cam), now)
      if not idx:
        continue
      for k in idx:
        payload = track.payload(cam, k)
        for v in viewers:
          v.push_frame(cam, payload, track.keys[k])
      self._sent[cam] = track.times[idx[-1]]

