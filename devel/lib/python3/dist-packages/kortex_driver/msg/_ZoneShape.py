# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/ZoneShape.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class ZoneShape(genpy.Message):
  _md5sum = "8de31189535cb06cb8caf5fc49d6c1b4"
  _type = "kortex_driver/ZoneShape"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
uint32 shape_type
Point origin
Base_RotationMatrix orientation
float32[] dimensions
float32 envelope_thickness
================================================================================
MSG: kortex_driver/Point

float32 x
float32 y
float32 z
================================================================================
MSG: kortex_driver/Base_RotationMatrix

Base_RotationMatrixRow row1
Base_RotationMatrixRow row2
Base_RotationMatrixRow row3
================================================================================
MSG: kortex_driver/Base_RotationMatrixRow

float32 column1
float32 column2
float32 column3"""
  __slots__ = ['shape_type','origin','orientation','dimensions','envelope_thickness']
  _slot_types = ['uint32','kortex_driver/Point','kortex_driver/Base_RotationMatrix','float32[]','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       shape_type,origin,orientation,dimensions,envelope_thickness

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ZoneShape, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.shape_type is None:
        self.shape_type = 0
      if self.origin is None:
        self.origin = kortex_driver.msg.Point()
      if self.orientation is None:
        self.orientation = kortex_driver.msg.Base_RotationMatrix()
      if self.dimensions is None:
        self.dimensions = []
      if self.envelope_thickness is None:
        self.envelope_thickness = 0.
    else:
      self.shape_type = 0
      self.origin = kortex_driver.msg.Point()
      self.orientation = kortex_driver.msg.Base_RotationMatrix()
      self.dimensions = []
      self.envelope_thickness = 0.

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
      buff.write(_get_struct_I12f().pack(_x.shape_type, _x.origin.x, _x.origin.y, _x.origin.z, _x.orientation.row1.column1, _x.orientation.row1.column2, _x.orientation.row1.column3, _x.orientation.row2.column1, _x.orientation.row2.column2, _x.orientation.row2.column3, _x.orientation.row3.column1, _x.orientation.row3.column2, _x.orientation.row3.column3))
      length = len(self.dimensions)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.dimensions))
      _x = self.envelope_thickness
      buff.write(_get_struct_f().pack(_x))
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
      if self.origin is None:
        self.origin = kortex_driver.msg.Point()
      if self.orientation is None:
        self.orientation = kortex_driver.msg.Base_RotationMatrix()
      end = 0
      _x = self
      start = end
      end += 52
      (_x.shape_type, _x.origin.x, _x.origin.y, _x.origin.z, _x.orientation.row1.column1, _x.orientation.row1.column2, _x.orientation.row1.column3, _x.orientation.row2.column1, _x.orientation.row2.column2, _x.orientation.row2.column3, _x.orientation.row3.column1, _x.orientation.row3.column2, _x.orientation.row3.column3,) = _get_struct_I12f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.dimensions = s.unpack(str[start:end])
      start = end
      end += 4
      (self.envelope_thickness,) = _get_struct_f().unpack(str[start:end])
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
      buff.write(_get_struct_I12f().pack(_x.shape_type, _x.origin.x, _x.origin.y, _x.origin.z, _x.orientation.row1.column1, _x.orientation.row1.column2, _x.orientation.row1.column3, _x.orientation.row2.column1, _x.orientation.row2.column2, _x.orientation.row2.column3, _x.orientation.row3.column1, _x.orientation.row3.column2, _x.orientation.row3.column3))
      length = len(self.dimensions)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.dimensions.tostring())
      _x = self.envelope_thickness
      buff.write(_get_struct_f().pack(_x))
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
      if self.origin is None:
        self.origin = kortex_driver.msg.Point()
      if self.orientation is None:
        self.orientation = kortex_driver.msg.Base_RotationMatrix()
      end = 0
      _x = self
      start = end
      end += 52
      (_x.shape_type, _x.origin.x, _x.origin.y, _x.origin.z, _x.orientation.row1.column1, _x.orientation.row1.column2, _x.orientation.row1.column3, _x.orientation.row2.column1, _x.orientation.row2.column2, _x.orientation.row2.column3, _x.orientation.row3.column1, _x.orientation.row3.column2, _x.orientation.row3.column3,) = _get_struct_I12f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.dimensions = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (self.envelope_thickness,) = _get_struct_f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_I12f = None
def _get_struct_I12f():
    global _struct_I12f
    if _struct_I12f is None:
        _struct_I12f = struct.Struct("<I12f")
    return _struct_I12f
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
