# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/CreateProtectionZoneRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class CreateProtectionZoneRequest(genpy.Message):
  _md5sum = "4d5a655d6317e3b457ac496f23c160cd"
  _type = "kortex_driver/CreateProtectionZoneRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """ProtectionZone input

================================================================================
MSG: kortex_driver/ProtectionZone

ProtectionZoneHandle handle
string name
string application_data
bool is_enabled
ZoneShape shape
CartesianLimitation[] limitations
CartesianLimitation[] envelope_limitations
================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/ZoneShape

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
float32 column3
================================================================================
MSG: kortex_driver/CartesianLimitation

uint32 type
float32 translation
float32 orientation"""
  __slots__ = ['input']
  _slot_types = ['kortex_driver/ProtectionZone']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       input

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CreateProtectionZoneRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.input is None:
        self.input = kortex_driver.msg.ProtectionZone()
    else:
      self.input = kortex_driver.msg.ProtectionZone()

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
      buff.write(_get_struct_2I().pack(_x.input.handle.identifier, _x.input.handle.permission))
      _x = self.input.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.input.application_data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_BI12f().pack(_x.input.is_enabled, _x.input.shape.shape_type, _x.input.shape.origin.x, _x.input.shape.origin.y, _x.input.shape.origin.z, _x.input.shape.orientation.row1.column1, _x.input.shape.orientation.row1.column2, _x.input.shape.orientation.row1.column3, _x.input.shape.orientation.row2.column1, _x.input.shape.orientation.row2.column2, _x.input.shape.orientation.row2.column3, _x.input.shape.orientation.row3.column1, _x.input.shape.orientation.row3.column2, _x.input.shape.orientation.row3.column3))
      length = len(self.input.shape.dimensions)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.input.shape.dimensions))
      _x = self.input.shape.envelope_thickness
      buff.write(_get_struct_f().pack(_x))
      length = len(self.input.limitations)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.limitations:
        _x = val1
        buff.write(_get_struct_I2f().pack(_x.type, _x.translation, _x.orientation))
      length = len(self.input.envelope_limitations)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.envelope_limitations:
        _x = val1
        buff.write(_get_struct_I2f().pack(_x.type, _x.translation, _x.orientation))
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
      if self.input is None:
        self.input = kortex_driver.msg.ProtectionZone()
      end = 0
      _x = self
      start = end
      end += 8
      (_x.input.handle.identifier, _x.input.handle.permission,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.input.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.input.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.input.application_data = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.input.application_data = str[start:end]
      _x = self
      start = end
      end += 53
      (_x.input.is_enabled, _x.input.shape.shape_type, _x.input.shape.origin.x, _x.input.shape.origin.y, _x.input.shape.origin.z, _x.input.shape.orientation.row1.column1, _x.input.shape.orientation.row1.column2, _x.input.shape.orientation.row1.column3, _x.input.shape.orientation.row2.column1, _x.input.shape.orientation.row2.column2, _x.input.shape.orientation.row2.column3, _x.input.shape.orientation.row3.column1, _x.input.shape.orientation.row3.column2, _x.input.shape.orientation.row3.column3,) = _get_struct_BI12f().unpack(str[start:end])
      self.input.is_enabled = bool(self.input.is_enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.input.shape.dimensions = s.unpack(str[start:end])
      start = end
      end += 4
      (self.input.shape.envelope_thickness,) = _get_struct_f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.limitations = []
      for i in range(0, length):
        val1 = kortex_driver.msg.CartesianLimitation()
        _x = val1
        start = end
        end += 12
        (_x.type, _x.translation, _x.orientation,) = _get_struct_I2f().unpack(str[start:end])
        self.input.limitations.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.envelope_limitations = []
      for i in range(0, length):
        val1 = kortex_driver.msg.CartesianLimitation()
        _x = val1
        start = end
        end += 12
        (_x.type, _x.translation, _x.orientation,) = _get_struct_I2f().unpack(str[start:end])
        self.input.envelope_limitations.append(val1)
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
      buff.write(_get_struct_2I().pack(_x.input.handle.identifier, _x.input.handle.permission))
      _x = self.input.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.input.application_data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_BI12f().pack(_x.input.is_enabled, _x.input.shape.shape_type, _x.input.shape.origin.x, _x.input.shape.origin.y, _x.input.shape.origin.z, _x.input.shape.orientation.row1.column1, _x.input.shape.orientation.row1.column2, _x.input.shape.orientation.row1.column3, _x.input.shape.orientation.row2.column1, _x.input.shape.orientation.row2.column2, _x.input.shape.orientation.row2.column3, _x.input.shape.orientation.row3.column1, _x.input.shape.orientation.row3.column2, _x.input.shape.orientation.row3.column3))
      length = len(self.input.shape.dimensions)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.input.shape.dimensions.tostring())
      _x = self.input.shape.envelope_thickness
      buff.write(_get_struct_f().pack(_x))
      length = len(self.input.limitations)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.limitations:
        _x = val1
        buff.write(_get_struct_I2f().pack(_x.type, _x.translation, _x.orientation))
      length = len(self.input.envelope_limitations)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.envelope_limitations:
        _x = val1
        buff.write(_get_struct_I2f().pack(_x.type, _x.translation, _x.orientation))
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
      if self.input is None:
        self.input = kortex_driver.msg.ProtectionZone()
      end = 0
      _x = self
      start = end
      end += 8
      (_x.input.handle.identifier, _x.input.handle.permission,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.input.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.input.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.input.application_data = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.input.application_data = str[start:end]
      _x = self
      start = end
      end += 53
      (_x.input.is_enabled, _x.input.shape.shape_type, _x.input.shape.origin.x, _x.input.shape.origin.y, _x.input.shape.origin.z, _x.input.shape.orientation.row1.column1, _x.input.shape.orientation.row1.column2, _x.input.shape.orientation.row1.column3, _x.input.shape.orientation.row2.column1, _x.input.shape.orientation.row2.column2, _x.input.shape.orientation.row2.column3, _x.input.shape.orientation.row3.column1, _x.input.shape.orientation.row3.column2, _x.input.shape.orientation.row3.column3,) = _get_struct_BI12f().unpack(str[start:end])
      self.input.is_enabled = bool(self.input.is_enabled)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.input.shape.dimensions = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (self.input.shape.envelope_thickness,) = _get_struct_f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.limitations = []
      for i in range(0, length):
        val1 = kortex_driver.msg.CartesianLimitation()
        _x = val1
        start = end
        end += 12
        (_x.type, _x.translation, _x.orientation,) = _get_struct_I2f().unpack(str[start:end])
        self.input.limitations.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.envelope_limitations = []
      for i in range(0, length):
        val1 = kortex_driver.msg.CartesianLimitation()
        _x = val1
        start = end
        end += 12
        (_x.type, _x.translation, _x.orientation,) = _get_struct_I2f().unpack(str[start:end])
        self.input.envelope_limitations.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_BI12f = None
def _get_struct_BI12f():
    global _struct_BI12f
    if _struct_BI12f is None:
        _struct_BI12f = struct.Struct("<BI12f")
    return _struct_BI12f
_struct_I2f = None
def _get_struct_I2f():
    global _struct_I2f
    if _struct_I2f is None:
        _struct_I2f = struct.Struct("<I2f")
    return _struct_I2f
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/CreateProtectionZoneResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class CreateProtectionZoneResponse(genpy.Message):
  _md5sum = "335f209b31742c233f4d4fd3cb08b30f"
  _type = "kortex_driver/CreateProtectionZoneResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """ProtectionZoneHandle output

================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
uint32 permission"""
  __slots__ = ['output']
  _slot_types = ['kortex_driver/ProtectionZoneHandle']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       output

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CreateProtectionZoneResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.output is None:
        self.output = kortex_driver.msg.ProtectionZoneHandle()
    else:
      self.output = kortex_driver.msg.ProtectionZoneHandle()

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
      buff.write(_get_struct_2I().pack(_x.output.identifier, _x.output.permission))
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
      if self.output is None:
        self.output = kortex_driver.msg.ProtectionZoneHandle()
      end = 0
      _x = self
      start = end
      end += 8
      (_x.output.identifier, _x.output.permission,) = _get_struct_2I().unpack(str[start:end])
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
      buff.write(_get_struct_2I().pack(_x.output.identifier, _x.output.permission))
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
      if self.output is None:
        self.output = kortex_driver.msg.ProtectionZoneHandle()
      end = 0
      _x = self
      start = end
      end += 8
      (_x.output.identifier, _x.output.permission,) = _get_struct_2I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
class CreateProtectionZone(object):
  _type          = 'kortex_driver/CreateProtectionZone'
  _md5sum = 'e5380d610764b507278413b8ea5bd27f'
  _request_class  = CreateProtectionZoneRequest
  _response_class = CreateProtectionZoneResponse
