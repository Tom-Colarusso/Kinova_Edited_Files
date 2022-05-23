# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/ToolConfiguration.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class ToolConfiguration(genpy.Message):
  _md5sum = "cf7e6c29cefe7625fb7412c1a3c76941"
  _type = "kortex_driver/ToolConfiguration"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
CartesianTransform tool_transform
float32 tool_mass
ControlConfig_Position tool_mass_center
================================================================================
MSG: kortex_driver/CartesianTransform

float32 x
float32 y
float32 z
float32 theta_x
float32 theta_y
float32 theta_z
================================================================================
MSG: kortex_driver/ControlConfig_Position

float32 x
float32 y
float32 z"""
  __slots__ = ['tool_transform','tool_mass','tool_mass_center']
  _slot_types = ['kortex_driver/CartesianTransform','float32','kortex_driver/ControlConfig_Position']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       tool_transform,tool_mass,tool_mass_center

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ToolConfiguration, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.tool_transform is None:
        self.tool_transform = kortex_driver.msg.CartesianTransform()
      if self.tool_mass is None:
        self.tool_mass = 0.
      if self.tool_mass_center is None:
        self.tool_mass_center = kortex_driver.msg.ControlConfig_Position()
    else:
      self.tool_transform = kortex_driver.msg.CartesianTransform()
      self.tool_mass = 0.
      self.tool_mass_center = kortex_driver.msg.ControlConfig_Position()

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
      buff.write(_get_struct_10f().pack(_x.tool_transform.x, _x.tool_transform.y, _x.tool_transform.z, _x.tool_transform.theta_x, _x.tool_transform.theta_y, _x.tool_transform.theta_z, _x.tool_mass, _x.tool_mass_center.x, _x.tool_mass_center.y, _x.tool_mass_center.z))
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
      if self.tool_transform is None:
        self.tool_transform = kortex_driver.msg.CartesianTransform()
      if self.tool_mass_center is None:
        self.tool_mass_center = kortex_driver.msg.ControlConfig_Position()
      end = 0
      _x = self
      start = end
      end += 40
      (_x.tool_transform.x, _x.tool_transform.y, _x.tool_transform.z, _x.tool_transform.theta_x, _x.tool_transform.theta_y, _x.tool_transform.theta_z, _x.tool_mass, _x.tool_mass_center.x, _x.tool_mass_center.y, _x.tool_mass_center.z,) = _get_struct_10f().unpack(str[start:end])
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
      buff.write(_get_struct_10f().pack(_x.tool_transform.x, _x.tool_transform.y, _x.tool_transform.z, _x.tool_transform.theta_x, _x.tool_transform.theta_y, _x.tool_transform.theta_z, _x.tool_mass, _x.tool_mass_center.x, _x.tool_mass_center.y, _x.tool_mass_center.z))
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
      if self.tool_transform is None:
        self.tool_transform = kortex_driver.msg.CartesianTransform()
      if self.tool_mass_center is None:
        self.tool_mass_center = kortex_driver.msg.ControlConfig_Position()
      end = 0
      _x = self
      start = end
      end += 40
      (_x.tool_transform.x, _x.tool_transform.y, _x.tool_transform.z, _x.tool_transform.theta_x, _x.tool_transform.theta_y, _x.tool_transform.theta_z, _x.tool_mass, _x.tool_mass_center.x, _x.tool_mass_center.y, _x.tool_mass_center.z,) = _get_struct_10f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10f = None
def _get_struct_10f():
    global _struct_10f
    if _struct_10f is None:
        _struct_10f = struct.Struct("<10f")
    return _struct_10f
