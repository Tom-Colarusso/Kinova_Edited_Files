# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/Base_CapSenseConfig.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Base_CapSenseConfig(genpy.Message):
  _md5sum = "f62e507319ac72b37a316fe4248e2e5a"
  _type = "kortex_driver/Base_CapSenseConfig"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
uint32 identifier
uint32 mode
float32 threshold_a
float32 threshold_b
float32 sensitivity_a
float32 sensitivity_b"""
  __slots__ = ['identifier','mode','threshold_a','threshold_b','sensitivity_a','sensitivity_b']
  _slot_types = ['uint32','uint32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       identifier,mode,threshold_a,threshold_b,sensitivity_a,sensitivity_b

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Base_CapSenseConfig, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.identifier is None:
        self.identifier = 0
      if self.mode is None:
        self.mode = 0
      if self.threshold_a is None:
        self.threshold_a = 0.
      if self.threshold_b is None:
        self.threshold_b = 0.
      if self.sensitivity_a is None:
        self.sensitivity_a = 0.
      if self.sensitivity_b is None:
        self.sensitivity_b = 0.
    else:
      self.identifier = 0
      self.mode = 0
      self.threshold_a = 0.
      self.threshold_b = 0.
      self.sensitivity_a = 0.
      self.sensitivity_b = 0.

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
      buff.write(_get_struct_2I4f().pack(_x.identifier, _x.mode, _x.threshold_a, _x.threshold_b, _x.sensitivity_a, _x.sensitivity_b))
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
      end = 0
      _x = self
      start = end
      end += 24
      (_x.identifier, _x.mode, _x.threshold_a, _x.threshold_b, _x.sensitivity_a, _x.sensitivity_b,) = _get_struct_2I4f().unpack(str[start:end])
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
      buff.write(_get_struct_2I4f().pack(_x.identifier, _x.mode, _x.threshold_a, _x.threshold_b, _x.sensitivity_a, _x.sensitivity_b))
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
      end = 0
      _x = self
      start = end
      end += 24
      (_x.identifier, _x.mode, _x.threshold_a, _x.threshold_b, _x.sensitivity_a, _x.sensitivity_b,) = _get_struct_2I4f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I4f = None
def _get_struct_2I4f():
    global _struct_2I4f
    if _struct_2I4f is None:
        _struct_2I4f = struct.Struct("<2I4f")
    return _struct_2I4f
