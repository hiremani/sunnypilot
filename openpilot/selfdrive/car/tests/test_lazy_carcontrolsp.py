"""LazyCarControlSP gives every field exactly the value convert_carControlSP gives it."""
import dataclasses
import unittest

from openpilot.cereal import messaging
from openpilot.selfdrive.car.helpers import LazyCarControlSP, convert_carControlSP


def reader(fill):
  m = messaging.new_message('carControlSP')
  fill(m.carControlSP)
  return messaging.log_from_bytes(m.to_bytes()).carControlSP


class TestLazyCarControlSP(unittest.TestCase):
  def test_every_field_matches(self):
    def fill(c):
      c.mads.active = True
      c.mads.available = True
      c.leadOne.dRel = 31.5
      c.leadOne.status = True
      c.leadTwo.vRel = -2.0
      c.intelligentCruiseButtonManagement.vTarget = 12.0
    for f in (lambda c: None, fill):
      r = reader(f)
      full = convert_carControlSP(r)
      lazy = LazyCarControlSP(r)
      for field in dataclasses.fields(full):
        if field.name.endswith("DEPRECATED") or field.name == "params":
          continue
        self.assertEqual(getattr(lazy, field.name), getattr(full, field.name), field.name)

  def test_reads_are_cached_and_unknown_names_raise(self):
    r = reader(lambda c: setattr(c.mads, "active", True))
    lazy = LazyCarControlSP(r)
    self.assertTrue(lazy.mads.active)
    self.assertIs(lazy.mads, lazy.mads)
    with self.assertRaises(AttributeError):
      _ = lazy.notAField


if __name__ == "__main__":
  unittest.main()
