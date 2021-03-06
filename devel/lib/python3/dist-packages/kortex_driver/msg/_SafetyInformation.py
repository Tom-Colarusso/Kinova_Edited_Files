# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/SafetyInformation.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class SafetyInformation(genpy.Message):
  _md5sum = "e8597ef9acfa23c653020b88d86d8b2f"
  _type = "kortex_driver/SafetyInformation"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
SafetyHandle handle
bool can_change_safety_state
bool has_warning_threshold
bool has_error_threshold
uint32 limit_type
float32 default_warning_threshold
float32 default_error_threshold
float32 upper_hard_limit
float32 lower_hard_limit
uint32 status
uint32 unit
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier"""
  __slots__ = ['handle','can_change_safety_state','has_warning_threshold','has_error_threshold','limit_type','default_warning_threshold','default_error_threshold','upper_hard_limit','lower_hard_limit','status','unit']
  _slot_types = ['kortex_driver/SafetyHandle','bool','bool','bool','uint32','float32','float32','float32','float32','uint32','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       handle,can_change_safety_state,has_warning_threshold,has_error_threshold,limit_type,default_warning_threshold,default_error_threshold,upper_hard_limit,lower_hard_limit,status,unit

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SafetyInformation, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.handle is None:
        self.handle = kortex_driver.msg.SafetyHandle()
      if self.can_change_safety_state is None:
        self.can_change_safety_state = False
      if self.has_warning_threshold is None:
        self.has_warning_threshold = False
      if self.has_error_threshold is None:
        self.has_error_threshold = False
      if self.limit_type is None:
        self.limit_type = 0
      if self.default_warning_threshold is None:
        self.default_warning_threshold = 0.
      if self.default_error_threshold is None:
        self.default_error_threshold = 0.
      if self.upper_hard_limit is None:
        self.upper_hard_limit = 0.
      if self.lower_hard_limit is None:
        self.lower_hard_limit = 0.
      if self.status is None:
        self.status = 0
      if self.unit is None:
        self.unit = 0
    else:
      self.handle = kortex_driver.msg.SafetyHandle()
      self.can_change_safety_state = False
      self.has_warning_threshold = False
      self.has_error_threshold = False
      self.limit_type = 0
      self.default_warning_threshold = 0.
      self.default_error_threshold = 0.
      self.upper_hard_limit = 0.
      self.lower_hard_limit = 0.
      self.status = 0
      self.unit = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_I3BI4f2I().pack(_x.handle.identifier, _x.can_change_safety_state, _x.has_warning_threshold, _x.has_error_threshold, _x.limit_type, _x.default_warning_threshold, _x.default_error_threshold, _x.upper_hard_limit, _x.lower_hard_limit, _x.status, _x.unit))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.handle is None:
        self.handle = kortex_driver.msg.SafetyHandle()
      end = 0
      _x = self
      start = end
      end += 35
      (_x.handle.identifier, _x.can_change_safety_state, _x.has_warning_threshold, _x.has_error_threshold, _x.limit_type, _x.default_warning_threshold, _x.default_error_threshold, _x.upper_hard_limit, _x.lower_hard_limit, _x.status, _x.unit,) = _get_struct_I3BI4f2I().unpack(str[start:end])
      self.can_change_safety_state = bool(self.can_change_safety_state)
      self.has_warning_threshold = bool(self.has_warning_threshold)
      self.has_error_threshold = bool(self.has_error_threshold)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_I3BI4f2I().pack(_x.handle.identifier, _x.can_change_safety_state, _x.has_warning_threshold, _x.has_error_threshold, _x.limit_type, _x.default_warning_threshold, _x.default_error_threshold, _x.upper_hard_limit, _x.lower_hard_limit, _x.status, _x.unit))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.handle is None:
        self.handle = kortex_driver.msg.SafetyHandle()
      end = 0
      _x = self
      start = end
      end += 35
      (_x.handle.identifier, _x.can_change_safety_state, _x.has_warning_threshold, _x.has_error_threshold, _x.limit_type, _x.default_warning_threshold, _x.default_error_threshold, _x.upper_hard_limit, _x.lower_hard_limit, _x.status, _x.unit,) = _get_struct_I3BI4f2I().unpack(str[start:end])
      self.can_change_safety_state = bool(self.can_change_safety_state)
      self.has_warning_threshold = bool(self.has_warning_threshold)
      self.has_error_threshold = bool(self.has_error_threshold)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_I3BI4f2I = None
def _get_struct_I3BI4f2I():
    global _struct_I3BI4f2I
    if _struct_I3BI4f2I is None:
        _struct_I3BI4f2I = struct.Struct("<I3BI4f2I")
    return _struct_I3BI4f2I
