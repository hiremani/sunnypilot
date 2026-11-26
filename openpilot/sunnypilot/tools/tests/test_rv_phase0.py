import unittest

from openpilot.sunnypilot.tools import rv_phase0 as p0


def obj_frame(oid, x, y, w=1.8, length=4.4, h=1.5, cls=0, conf=1, class_conf=2):
  """An ADAS_ObjN frame as the ADAS module sends it (Motorola, 0.2 m/bit), for decode_obj to read back."""
  b = bytearray(8)
  b[0] = oid
  b[1] = round(abs(x) / 0.2)
  b[2] = round(abs(y) / 0.2)
  b[3] = (round(w / 0.1) & 0x3F) | (conf << 7)
  b[4] = round(length / 0.2)
  b[6] = (round(h / 0.1) & 0x3F) | ((x < 0) << 6) | ((y > 0) << 7)   # long sign 1 = behind; lat sign 1 = left
  b[7] = (cls & 0x0F) | (class_conf << 5)
  return bytes(b)


class TestDecode(unittest.TestCase):
  def test_empty_slot(self):
    self.assertIsNone(p0.decode_obj(bytes(8)))
    self.assertIsNone(p0.decode_obj(b"\x05\x01"))   # short frame

  def test_ahead_right(self):
    o = p0.decode_obj(obj_frame(7, 24.4, -1.8, cls=1))
    self.assertEqual((o["id"], o["x"], o["y"], o["cls"]), (7, 24.4, -1.8, "truck"))
    self.assertAlmostEqual(o["w"], 1.8)
    self.assertAlmostEqual(o["l"], 4.4)
    self.assertAlmostEqual(o["h"], 1.5)
    self.assertEqual((o["conf"], o["class_conf"]), (1, 2))

  def test_behind_left(self):
    o = p0.decode_obj(obj_frame(3, -12.0, 3.6, cls=4))
    self.assertEqual((o["x"], o["y"], o["cls"]), (-12.0, 3.6, "pedestrian"))


class TestHelpers(unittest.TestCase):
  def test_speed_bucket(self):
    self.assertEqual([p0.speed_bucket(v) for v in (0.2, 10, 20, 30)], ["parked", "town", "road", "highway"])

  def test_percentiles(self):
    s = p0.percentiles(range(101))
    self.assertEqual((s["n"], s["min"], s["max"], s["p50"], s["p95"]), (101, 0, 100, 50, 95))
    self.assertIsNone(p0.percentiles([]))
    self.assertEqual(p0.percentiles([None, float("nan"), 3])["n"], 1)

  def test_spread(self):
    self.assertEqual(p0.spread(list(range(10)), 3), [0, 4, 9])   # first, middle, last
    self.assertEqual(p0.spread([1, 2], 6), [1, 2])
    self.assertEqual(p0.spread(list(range(5)), 1), [2])


class TestTracks(unittest.TestCase):
  def test_a_car_that_passes_us_is_kept_behind(self):
    tr = p0.Tracks()
    # ahead at 20 m, alongside, then behind for 1.5 s to -8 m
    for i, x in enumerate([20, 12, 6, 2, -1, -4, -8]):
      tr.see(i * 0.5, {"id": 9, "x": x, "cls": "car"})
    (k,) = tr.close()
    self.assertTrue(k["ahead_seen"])
    self.assertEqual(k["behind_t0"], 2.0)
    self.assertAlmostEqual(k["behind_s"], 1.0)
    self.assertEqual(k["xmin"], -8)

  def test_a_gap_starts_a_new_track(self):
    tr = p0.Tracks()
    tr.see(0.0, {"id": 4, "x": 10, "cls": "car"})
    tr.see(0.5, {"id": 4, "x": 9, "cls": "car"})
    tr.see(3.0, {"id": 4, "x": 30, "cls": "car"})   # the ID came back after 2.5 s: another object
    self.assertEqual(len(tr.close()), 2)

  def test_first_seen_behind_never_counts_as_passed(self):
    tr = p0.Tracks()
    for i, x in enumerate([-6, -5, -4]):
      tr.see(i * 0.1, {"id": 2, "x": x, "cls": "car"})
    (k,) = tr.close()
    self.assertIsNone(k["behind_t0"])
    self.assertEqual(k["x0"], -6)


class TestMergeSummary(unittest.TestCase):
  def acc(self, behind, ahead, lead_radar):
    a = p0.new_acc()
    a["segments"] = 1
    a["drives"] = {"r1"}
    g = a["obj"]["town"]
    g["n"] = behind + ahead
    g["behind"] = behind
    g["x_behind"] = [-5.0] * behind
    g["x_ahead"] = [20.0] * ahead
    a["snapshots"]["town"] = 10
    a["slots"][2] = 10
    a["lead"] = {"present": 10, "radar": lead_radar}
    a["model"]["nostream"]["exec_ms"] = [20.0, 30.0]
    return a

  def test_merge_adds_counts_and_lists(self):
    m = p0.merge(self.acc(2, 8, 0), self.acc(0, 10, 5))
    self.assertEqual(m["segments"], 2)
    self.assertEqual(m["obj"]["town"]["n"], 20)
    self.assertEqual(len(m["obj"]["town"]["x_ahead"]), 18)
    self.assertEqual(m["slots"][2], 20)
    s = p0.summarize(m)
    self.assertEqual(s["adas_objects"]["by_speed"]["town"]["behind_share"], 0.1)
    self.assertEqual(s["leads"]["radar_backed_share"], 0.25)
    self.assertEqual(s["model"]["nostream"]["model_exec_ms"]["n"], 4)
    self.assertEqual(s["drives"], 1)

  def test_markdown_renders(self):
    s = p0.summarize(self.acc(1, 9, 0))
    md = p0.markdown(s, ["r1"])
    self.assertIn("ADAS object list", md)
    self.assertIn("radar-backed share 0.0", md)


if __name__ == "__main__":
  unittest.main()
