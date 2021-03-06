# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/JointAngles.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class JointAngles(genpy.Message):
  _md5sum = "94886d22261db2d621b5fe4c4bffdfa1"
  _type = "kortex_driver/JointAngles"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
float32 value"""
  __slots__ = ['joint_angles']
  _slot_types = ['kortex_driver/JointAngle[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       joint_angles

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(JointAngles, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.joint_angles is None:
        self.joint_angles = []
    else:
      self.joint_angles = []

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
      length = len(self.joint_angles)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_angles:
        _x = val1
        buff.write(_get_struct_If().pack(_x.joint_identifier, _x.value))
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
      if self.joint_angles is None:
        self.joint_angles = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_angles = []
      for i in range(0, length):
        val1 = kortex_driver.msg.JointAngle()
        _x = val1
        start = end
        end += 8
        (_x.joint_identifier, _x.value,) = _get_struct_If().unpack(str[start:end])
        self.joint_angles.append(val1)
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
      length = len(self.joint_angles)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_angles:
        _x = val1
        buff.write(_get_struct_If().pack(_x.joint_identifier, _x.value))
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
      if self.joint_angles is None:
        self.joint_angles = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_angles = []
      for i in range(0, length):
        val1 = kortex_driver.msg.JointAngle()
        _x = val1
        start = end
        end += 8
        (_x.joint_identifier, _x.value,) = _get_struct_If().unpack(str[start:end])
        self.joint_angles.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_If = None
def _get_struct_If():
    global _struct_If
    if _struct_If is None:
        _struct_If = struct.Struct("<If")
    return _struct_If
