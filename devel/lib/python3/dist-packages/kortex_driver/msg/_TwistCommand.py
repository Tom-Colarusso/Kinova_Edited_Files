# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/TwistCommand.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class TwistCommand(genpy.Message):
  _md5sum = "16dcfd20a022a10eea1f05e5a9cbb18a"
  _type = "kortex_driver/TwistCommand"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
uint32 reference_frame
Twist twist
uint32 duration
================================================================================
MSG: kortex_driver/Twist

float32 linear_x
float32 linear_y
float32 linear_z
float32 angular_x
float32 angular_y
float32 angular_z"""
  __slots__ = ['reference_frame','twist','duration']
  _slot_types = ['uint32','kortex_driver/Twist','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       reference_frame,twist,duration

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TwistCommand, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.reference_frame is None:
        self.reference_frame = 0
      if self.twist is None:
        self.twist = kortex_driver.msg.Twist()
      if self.duration is None:
        self.duration = 0
    else:
      self.reference_frame = 0
      self.twist = kortex_driver.msg.Twist()
      self.duration = 0

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
      buff.write(_get_struct_I6fI().pack(_x.reference_frame, _x.twist.linear_x, _x.twist.linear_y, _x.twist.linear_z, _x.twist.angular_x, _x.twist.angular_y, _x.twist.angular_z, _x.duration))
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
      if self.twist is None:
        self.twist = kortex_driver.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 32
      (_x.reference_frame, _x.twist.linear_x, _x.twist.linear_y, _x.twist.linear_z, _x.twist.angular_x, _x.twist.angular_y, _x.twist.angular_z, _x.duration,) = _get_struct_I6fI().unpack(str[start:end])
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
      buff.write(_get_struct_I6fI().pack(_x.reference_frame, _x.twist.linear_x, _x.twist.linear_y, _x.twist.linear_z, _x.twist.angular_x, _x.twist.angular_y, _x.twist.angular_z, _x.duration))
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
      if self.twist is None:
        self.twist = kortex_driver.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 32
      (_x.reference_frame, _x.twist.linear_x, _x.twist.linear_y, _x.twist.linear_z, _x.twist.angular_x, _x.twist.angular_y, _x.twist.angular_z, _x.duration,) = _get_struct_I6fI().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_I6fI = None
def _get_struct_I6fI():
    global _struct_I6fI
    if _struct_I6fI is None:
        _struct_I6fI = struct.Struct("<I6fI")
    return _struct_I6fI
