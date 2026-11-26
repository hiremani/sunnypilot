# distutils: language = c++
# cython: language_level=3
"""Native 'can' / 'sendcan' packing and unpacking for card (see can_codec.h and pandad_api_impl.py)."""
from libcpp cimport bool

cdef extern from "selfdrive/pandad/can_codec.h":
  object can_event_to_py(const char* data, size_t size, bool sendcan)
  object can_list_to_bytes(object msgs, bool sendcan, bool valid, unsigned long long log_mono_time)


def event_to_list(bytes s, bint sendcan):
  """(logMonoTime, [(address, dat, src), ...]) or None when not handled."""
  return can_event_to_py(s, len(s), sendcan)


def list_to_event(msgs, bint sendcan, bint valid, unsigned long long log_mono_time):
  """Serialized Event bytes, or None when not handled."""
  return can_list_to_bytes(msgs, sendcan, valid, log_mono_time)
