import capnp
import dataclasses
from typing import Any

from openpilot.cereal import custom
from opendbc.car import structs

_FIELDS = '__dataclass_fields__'  # copy of dataclasses._FIELDS


def is_dataclass(obj):
  """Similar to dataclasses.is_dataclass without instance type check checking"""
  return hasattr(obj, _FIELDS)


def _asdictref_inner(obj) -> dict[str, Any] | Any:
  if is_dataclass(obj):
    ret = {}
    for field in getattr(obj, _FIELDS):  # similar to dataclasses.fields()
      ret[field] = _asdictref_inner(getattr(obj, field))
    return ret
  elif isinstance(obj, (tuple, list)):
    return type(obj)(_asdictref_inner(v) for v in obj)
  else:
    return obj


def asdictref(obj) -> dict[str, Any]:
  """
  Similar to dataclasses.asdict without recursive type checking and copy.deepcopy
  Note that the resulting dict will contain references to the original struct as a result
  """
  if not is_dataclass(obj):
    raise TypeError("asdictref() should be called on dataclass instances")

  return _asdictref_inner(obj)


def convert_to_capnp(struct: structs.CarParamsSP | structs.CarStateSP) -> capnp.lib.capnp._DynamicStructBuilder:
  struct_dict = asdictref(struct)

  if isinstance(struct, structs.CarParamsSP):
    struct_capnp = custom.CarParamsSP.new_message(**struct_dict)
  elif isinstance(struct, structs.CarStateSP):
    struct_capnp = custom.CarStateSP.new_message(**struct_dict)
  else:
    raise ValueError(f"Unsupported struct type: {type(struct)}")

  return struct_capnp


class LazyCarControlSP:
  """convert_carControlSP's result, converted one field at a time as the car code reads it (and then cached).

  card converts carControlSP on every 100 Hz tick, and capnp's to_dict() of the whole struct took ~0.6 ms of the
  device's CPU per tick (16% of Fisker card's tick on route 000000f6) when the car controller reads one field
  (mads.active). Each field gets exactly the value convert_carControlSP gives it."""
  _SUBS = {"mads": structs.ModularAssistiveDrivingSystem, "leadOne": structs.LeadData, "leadTwo": structs.LeadData,
           "intelligentCruiseButtonManagement": structs.IntelligentCruiseButtonManagement}

  def __init__(self, struct: capnp.lib.capnp._DynamicStructReader):
    self._struct = struct

  def __getattr__(self, name: str):
    if name.startswith("_"):
      raise AttributeError(name)
    if name in self._SUBS:
      # field by field: the sub-struct's own to_dict() leaves out fields at their capnp default, which would then take the
      # dataclass's default instead (LeadData.radarTrackId: -1, not 0)
      # (and a sub-struct that was never set is left out of to_dict() entirely: all the dataclass's defaults)
      cls = self._SUBS[name]
      if not self._struct._has(name):
        value = cls()
      else:
        sub = getattr(self._struct, name)
        value = cls(**{f.name: _plain(getattr(sub, f.name)) for f in dataclasses.fields(cls) if not f.name.endswith("DEPRECATED")})
    elif name in _CC_SP_FIELDS:
      value = getattr(self._struct, name)
      if not isinstance(value, (bool, int, float, str)):
        value = getattr(convert_carControlSP(self._struct), name)   # anything else: the full conversion's value
    else:
      raise AttributeError(name)
    self.__dict__[name] = value
    return value


_CC_SP_FIELDS = {f.name for f in dataclasses.fields(structs.CarControlSP)}


def _plain(v):
  return v if isinstance(v, (bool, int, float, str)) else str(v)   # enums read as their name, as to_dict() gives them


def convert_carControlSP(struct: capnp.lib.capnp._DynamicStructReader) -> structs.CarControlSP:
  # TODO: recursively handle any car struct as needed
  def remove_deprecated(s: dict) -> dict:
    return {k: v for k, v in s.items() if not k.endswith('DEPRECATED')}

  struct_dict = struct.to_dict()
  struct_dataclass = structs.CarControlSP(**remove_deprecated({k: v for k, v in struct_dict.items() if not isinstance(k, dict)}))

  struct_dataclass.mads = structs.ModularAssistiveDrivingSystem(**remove_deprecated(struct_dict.get('mads', {})))
  # struct_dataclass.params = [structs.CarControlSP.Param(**remove_deprecated(p)) for p in struct_dict.get('params', [])]
  struct_dataclass.leadOne = structs.LeadData(**remove_deprecated(struct_dict.get('leadOne', {})))
  struct_dataclass.leadTwo = structs.LeadData(**remove_deprecated(struct_dict.get('leadTwo', {})))
  struct_dataclass.intelligentCruiseButtonManagement = structs.IntelligentCruiseButtonManagement(
    **remove_deprecated(struct_dict.get('intelligentCruiseButtonManagement', {}))
  )

  return struct_dataclass
