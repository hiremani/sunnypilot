// The 'can' / 'sendcan' event <-> Python lists, in C++ (the same results as pandad_api_impl.py's Python functions,
// which stay the reference and the fallback). Both return a new reference, or Py_None when this code does not handle
// the input (wrong event type, odd Python types, a capnp error): the caller then runs the Python function, which
// gives the same result or raises its usual error.
#pragma once

#include <Python.h>

#include <cstdint>
#include <cstring>

#include <capnp/message.h>
#include <capnp/serialize.h>

#include "openpilot/cereal/gen/cpp/log.capnp.h"

static inline PyObject* not_handled() {
  PyErr_Clear();
  Py_RETURN_NONE;
}

// one serialized Event -> (logMonoTime, [(address, dat, src), ...])
static PyObject* can_event_to_py(const char* data, size_t size, bool sendcan) {
  try {
    size_t nwords = size / sizeof(capnp::word);
    kj::Array<capnp::word> copy;
    kj::ArrayPtr<const capnp::word> words;
    if ((reinterpret_cast<uintptr_t>(data) & (sizeof(capnp::word) - 1)) == 0) {
      words = kj::arrayPtr(reinterpret_cast<const capnp::word*>(data), nwords);
    } else {
      copy = kj::heapArray<capnp::word>(nwords);
      memcpy(copy.begin(), data, nwords * sizeof(capnp::word));
      words = copy;
    }
    capnp::ReaderOptions options;
    options.traversalLimitInWords = kj::maxValue;
    capnp::FlatArrayMessageReader reader(words, options);
    cereal::Event::Reader event = reader.getRoot<cereal::Event>();
    if (event.which() != (sendcan ? cereal::Event::SENDCAN : cereal::Event::CAN)) return not_handled();
    auto frames = sendcan ? event.getSendcan() : event.getCan();

    PyObject* list = PyList_New(frames.size());
    if (list == nullptr) return nullptr;
    for (unsigned i = 0; i < frames.size(); i++) {
      auto f = frames[i];
      auto dat = f.getDat();
      PyObject* item = Py_BuildValue("(ky#B)", static_cast<unsigned long>(f.getAddress()),
                                     reinterpret_cast<const char*>(dat.begin()), static_cast<Py_ssize_t>(dat.size()),
                                     static_cast<unsigned char>(f.getSrc()));
      if (item == nullptr) { Py_DECREF(list); return nullptr; }
      PyList_SET_ITEM(list, i, item);
    }
    PyObject* nanos = PyLong_FromUnsignedLongLong(event.getLogMonoTime());
    if (nanos == nullptr) { Py_DECREF(list); return nullptr; }
    PyObject* ret = PyTuple_Pack(2, nanos, list);
    Py_DECREF(nanos);
    Py_DECREF(list);
    return ret;
  } catch (...) {
    return not_handled();
  }
}

// [(address, dat, src), ...] -> one serialized Event (bytes)
static PyObject* can_list_to_bytes(PyObject* msgs, bool sendcan, bool valid, uint64_t log_mono_time) {
  PyObject* seq = PySequence_Fast(msgs, "can messages");
  if (seq == nullptr) return not_handled();
  PyObject* ret = nullptr;
  try {
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    capnp::MallocMessageBuilder msg;
    cereal::Event::Builder event = msg.initRoot<cereal::Event>();
    event.setValid(valid);
    event.setLogMonoTime(log_mono_time);
    auto frames = sendcan ? event.initSendcan(n) : event.initCan(n);
    bool ok = true;
    for (Py_ssize_t i = 0; i < n && ok; i++) {
      PyObject* m = PySequence_Fast_GET_ITEM(seq, i);
      if (!PyTuple_Check(m) || PyTuple_GET_SIZE(m) < 3) { ok = false; break; }
      PyObject* a = PyTuple_GET_ITEM(m, 0);
      PyObject* d = PyTuple_GET_ITEM(m, 1);
      PyObject* s = PyTuple_GET_ITEM(m, 2);
      if (!PyLong_CheckExact(a) || !PyBytes_CheckExact(d) || !PyLong_CheckExact(s)) { ok = false; break; }
      unsigned long long addr = PyLong_AsUnsignedLongLong(a);
      unsigned long long src = PyLong_AsUnsignedLongLong(s);
      if (PyErr_Occurred() || addr > 0xFFFFFFFFULL || src > 0xFFULL) { ok = false; break; }
      auto f = frames[i];
      f.setAddress(static_cast<uint32_t>(addr));
      f.setDat(kj::arrayPtr(reinterpret_cast<const kj::byte*>(PyBytes_AS_STRING(d)), PyBytes_GET_SIZE(d)));
      f.setSrc(static_cast<uint8_t>(src));
    }
    if (ok) {
      auto words = capnp::messageToFlatArray(msg);
      auto bytes = words.asBytes();
      ret = PyBytes_FromStringAndSize(reinterpret_cast<const char*>(bytes.begin()), bytes.size());
    }
  } catch (...) {
    ret = nullptr;
  }
  Py_DECREF(seq);
  if (ret == nullptr) return not_handled();
  return ret;
}
