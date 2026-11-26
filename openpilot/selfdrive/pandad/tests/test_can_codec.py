"""The native 'can' / 'sendcan' packing and unpacking (can_codec_pyx) must give what the Python functions give."""
import random
import unittest

import capnp

from opendbc.car.can_definitions import CanData
from openpilot.selfdrive.pandad import pandad_api_impl as impl


@unittest.skipIf(impl._native is None, "can_codec_pyx not built (scons)")
class TestCanCodec(unittest.TestCase):
  def frames(self, rng, n):
    return [(rng.choice([rng.randrange(0x800), rng.randrange(1 << 29), 0xFFFFFFFF]),
             bytes(rng.randrange(256) for _ in range(rng.choice([0, 1, 3, 8, 8, 8, 12, 64]))), rng.randrange(256)) for _ in range(n)]

  def test_pack_is_byte_identical(self):
    rng = random.Random(1)
    for _ in range(300):
      msgs = self.frames(rng, rng.choice([0, 1, 5, 23, 120, 400]))
      for msgtype in ('can', 'sendcan'):
        for valid in (True, False):
          t = rng.randrange(1 << 62)
          native = impl._native.list_to_event(msgs, msgtype == 'sendcan', valid, t)
          self.assertEqual(native, impl.can_list_to_can_capnp_py(msgs, msgtype, valid, t))

  def test_unpack_matches(self):
    rng = random.Random(2)
    for _ in range(300):
      msgtype = rng.choice(['can', 'sendcan'])
      strings = [impl.can_list_to_can_capnp_py(self.frames(rng, rng.choice([0, 1, 30, 150])), msgtype, True, rng.randrange(1 << 62))
                 for _ in range(rng.randrange(0, 4))]
      got = impl.can_capnp_to_list(strings, msgtype)
      want = impl.can_capnp_to_list_py(strings, msgtype)
      self.assertEqual(got, want)
      for (_, fa), (_, fb) in zip(got, want, strict=True):
        self.assertEqual([tuple(map(type, f)) for f in fa], [tuple(map(type, f)) for f in fb])

  def test_round_trip_and_named_tuples(self):
    msgs = [CanData(0x1D0, b"\x01\x02\x03\x04\x05\x06\x07\x08", 0), CanData(0x5D0, b"", 1)]
    out = impl.can_list_to_can_capnp(msgs, msgtype='sendcan')
    self.assertEqual(impl.can_capnp_to_list([out], 'sendcan')[0][1], [tuple(m) for m in msgs])

  def test_inputs_the_native_code_leaves_to_python(self):
    # a bytearray payload, a list instead of a tuple, an event of the other type: the result is the Python function's
    self.assertIsNone(impl._native.list_to_event([(1, bytearray(b"ab"), 0)], False, True, 5))
    self.assertIsNone(impl._native.list_to_event([[1, b"ab", 0]], False, True, 5))
    self.assertIsNone(impl._native.list_to_event([(1 << 40, b"ab", 0)], False, True, 5))
    ok = impl.can_list_to_can_capnp([[1, b"ab", 0]])
    self.assertEqual(impl.can_capnp_to_list([ok])[0][1], [(1, b"ab", 0)])
    can = impl.can_list_to_can_capnp_py([(1, b"ab", 0)], 'can', True, 5)
    self.assertIsNone(impl._native.event_to_list(can, True))
    with self.assertRaises(capnp.lib.capnp.KjException):
      impl.can_capnp_to_list([can], 'sendcan')
    with self.assertRaises(capnp.lib.capnp.KjException):
      impl.can_capnp_to_list_py([can], 'sendcan')
    self.assertIsNone(impl._native.event_to_list(b"\x00\x01\x02", False))


if __name__ == "__main__":
  unittest.main()
