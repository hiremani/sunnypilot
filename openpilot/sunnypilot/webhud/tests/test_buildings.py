"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os
import tempfile
import time
import unittest

from openpilot.sunnypilot.webhud.buildings import (BLD_DEG, AreaDownload, MapBuildings, cell_bbox, cells_within, parse_path,
                                                   valid_cell)

ANSWER = b'{"version":0.6,"elements":[{"type":"way","id":1,"tags":{"building":"house"},"geometry":[]}]}'


class StubFetch:
  def __init__(self, answer=ANSWER, fail=False):
    self.queries: list[str] = []
    self.answer = answer
    self.fail = fail

  def __call__(self, query: str) -> bytes:
    self.queries.append(query)
    if self.fail:
      raise OSError("offline")
    return self.answer


class TestCells(unittest.TestCase):
  def test_path_parsing(self):
    assert parse_path("/map/buildings/3752/-12198") == (3752, -12198)
    assert parse_path("/map/buildings/3752") is None
    assert parse_path("/map/features/1/2") is None
    assert parse_path("/map/buildings/a/b") is None
    assert parse_path("/map/buildings/9000/0") is None   # past the pole
    assert valid_cell(-9000, -18000) and not valid_cell(0, 18000)

  def test_bbox_matches_the_app_and_serve_py(self):
    # the same "%.4f" south, west, north, east the Pulse app (MapFeatures.kt) and serve.py send
    assert cell_bbox(3752, -12198) == "37.5200,-121.9800,37.5300,-121.9700"

  def test_cells_within_nearest_first(self):
    lat, lon = 37.5255, -121.9855
    cells = cells_within(lat, lon, 1500)
    assert cells[0] == (3752, -12199)   # the cell the point is in comes first
    assert len(cells) == len(set(cells))
    # every cell's box is within the radius (about 1.5 km = 0.0135 deg of latitude)
    for klat, _ in cells:
      assert abs((klat + 0.5) * BLD_DEG - lat) < 0.03
    assert cells_within(lat, lon, 0) == [(3752, -12199)]


class TestMapBuildings(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.fetch = StubFetch()
    self.b = MapBuildings(self.tmp.name, fetch=self.fetch)

  def tearDown(self):
    self.tmp.cleanup()

  def test_fetched_once_then_from_disk(self):
    p = self.b.cell_path(3752, -12198)
    assert p is not None
    with open(p, "rb") as f:
      assert f.read() == ANSWER
    assert len(self.fetch.queries) == 1
    q = self.fetch.queries[0]
    assert 'way["building"](37.5200,-121.9800,37.5300,-121.9700)' in q and "out tags geom qt;" in q
    assert self.b.cell_path(3752, -12198) == p
    assert len(self.fetch.queries) == 1

  def test_no_fetching_while_moving_but_disk_still_served(self):
    self.b.cell_path(1, 1)
    self.b.allow_fetch = lambda: False
    assert self.b.cell_path(2, 2) is None
    assert len(self.fetch.queries) == 1
    assert self.b.cell_path(1, 1) is not None   # what's on disk is served while driving too

  def test_a_failed_cell_waits_before_it_is_tried_again(self):
    self.fetch.fail = True
    assert self.b.cell_path(5, 5) is None
    assert self.b.cell_path(5, 5) is None
    assert len(self.fetch.queries) == 1
    assert not os.listdir(self.tmp.name) or not any(n.endswith(".json") for n in os.listdir(self.tmp.name))

  def test_a_non_json_answer_is_not_kept(self):
    self.fetch.answer = b"<html>rate limited</html>"
    assert self.b.cell_path(6, 6) is None
    assert not self.b.have(6, 6)

  def test_folder_is_trimmed_oldest_read_first(self):
    self.b.max_bytes = len(ANSWER) * 2
    self.b.cell_path(1, 1)
    time.sleep(0.02)
    self.b.cell_path(2, 2)
    time.sleep(0.02)
    self.b.cell_path(1, 1)   # read again: now the newest
    time.sleep(0.02)
    self.b.cell_path(3, 3)   # over the bound: (2, 2) was read longest ago
    assert self.b.have(1, 1) and self.b.have(3, 3) and not self.b.have(2, 2)

  def test_prefetch_nearest_first_resumes_and_stops(self):
    lat, lon = 37.5255, -121.9855
    n, errors = self.b.prefetch(lat, lon, 0.5, keep_going=lambda: len(self.fetch.queries) < 2, pause_s=0)
    assert errors == 0 and len(self.fetch.queries) == 2 and n > 2
    assert self.b.have(*cells_within(lat, lon, 500)[0])
    n2, _ = self.b.prefetch(lat, lon, 0.5, pause_s=0)   # resumes: the two on disk aren't fetched again
    assert len(self.fetch.queries) == n2
    assert self.b.ready(lat, lon, 0.5) == (n2, n2)

  def test_prefetch_radius_is_bounded(self):
    n, _ = self.b.prefetch(37.5, -122.0, 1000, keep_going=lambda: False, pause_s=0)
    assert n == len(cells_within(37.5, -122.0, 10_000))

  def test_area_download_runs_in_the_background(self):
    area = AreaDownload(self.b)
    area.pause_s = 0
    st = area.start(37.5255, -121.9855, 0.3)
    assert st["center"] == [37.5255, -121.9855] and st["cells"] > 0
    for _ in range(200):
      st = area.status()
      if not st["busy"] and st["ready"] == st["cells"]:
        break
      time.sleep(0.05)
    assert st["ready"] == st["cells"] and st["errors"] == 0
    with self.assertRaises(ValueError):
      area.start(95, 0, 1)


if __name__ == "__main__":
  unittest.main()
