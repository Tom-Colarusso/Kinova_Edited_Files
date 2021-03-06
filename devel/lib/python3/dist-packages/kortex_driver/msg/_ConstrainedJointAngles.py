# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/ConstrainedJointAngles.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class ConstrainedJointAngles(genpy.Message):
  _md5sum = "75faad16493b9d4c290ef3b7fb5a8947"
  _type = "kortex_driver/ConstrainedJointAngles"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
JointAngles joint_angles
JointTrajectoryConstraint constraint
================================================================================
MSG: kortex_driver/JointAngles

JointAngle[] joint_angles
================================================================================
MSG: kortex_driver/JointAngle

uint32 joint_identifier
float32 value
================================================================================
MSG: kortex_driver/JointTrajectoryConstraint

uint32 type
float32 value"""
  __slots__ = ['joint_angles','constraint']
  _slot_types = ['kortex_driver/JointAngles','kortex_driver/JointTrajectoryConstraint']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       joint_angles,constraint

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ConstrainedJointAngles, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.joint_angles is None:
        self.joint_angles = kortex_driver.msg.JointAngles()
      if self.constraint is None:
        self.constraint = kortex_driver.msg.JointTrajectoryConstraint()
    else:
      self.joint_angles = kortex_driver.msg.JointAngles()
      self.constraint = kortex_driver.msg.JointTrajectoryConstraint()

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
      length = len(self.joint_angles.joint_angles)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_angles.joint_angles:
        _x = val1
        buff.write(_get_struct_If().pack(_x.joint_identifier, _x.value))
      _x = self
      buff.write(_get_struct_If().pack(_x.constraint.type, _x.constraint.value))
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
        self.joint_angles = kortex_driver.msg.JointAngles()
      if self.constraint is None:
        self.constraint = kortex_driver.msg.JointTrajectoryConstraint()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_angles.joint_angles = []
      for i in range(0, length):
        val1 = kortex_driver.msg.JointAngle()
        _x = val1
        start = end
        end += 8
        (_x.joint_identifier, _x.value,) = _get_struct_If().unpack(str[start:end])
        self.joint_angles.joint_angles.append(val1)
      _x = self
      start = end
      end += 8
      (_x.constraint.type, _x.constraint.value,) = _get_struct_If().unpack(str[start:end])
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
      length = len(self.joint_angles.joint_angles)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_angles.joint_angles:
        _x = val1
        buff.write(_get_struct_If().pack(_x.joint_identifier, _x.value))
      _x = self
      buff.write(_get_struct_If().pack(_x.constraint.type, _x.constraint.value))
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
        self.joint_angles = kortex_driver.msg.JointAngles()
      if self.constraint is None:
        self.constraint = kortex_driver.msg.JointTrajectoryConstraint()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_angles.joint_angles = []
      for i in range(0, length):
        val1 = kortex_driver.msg.JointAngle()
        _x = val1
        start = end
        end += 8
        (_x.joint_identifier, _x.value,) = _get_struct_If().unpack(str[start:end])
        self.joint_angles.joint_angles.append(val1)
      _x = self
      start = end
      end += 8
      (_x.constraint.type, _x.constraint.value,) = _get_struct_If().unpack(str[start:end])
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
