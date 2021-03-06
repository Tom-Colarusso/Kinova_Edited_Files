# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/ExecuteCalibrationRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class ExecuteCalibrationRequest(genpy.Message):
  _md5sum = "68745c1f95256ccea9f0848f17f7fa0a"
  _type = "kortex_driver/ExecuteCalibrationRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """Calibration input

================================================================================
MSG: kortex_driver/Calibration

uint32 calibration_item
CalibrationParameter[] calibration_parameter
================================================================================
MSG: kortex_driver/CalibrationParameter

uint32 calibration_parameter_identifier
CalibrationParameter_value oneof_value
================================================================================
MSG: kortex_driver/CalibrationParameter_value

uint32[] signedIntValue
uint32[] unsignedIntValue
uint32[] floatValue"""
  __slots__ = ['input']
  _slot_types = ['kortex_driver/Calibration']

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
      super(ExecuteCalibrationRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.input is None:
        self.input = kortex_driver.msg.Calibration()
    else:
      self.input = kortex_driver.msg.Calibration()

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
      _x = self.input.calibration_item
      buff.write(_get_struct_I().pack(_x))
      length = len(self.input.calibration_parameter)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.calibration_parameter:
        _x = val1.calibration_parameter_identifier
        buff.write(_get_struct_I().pack(_x))
        _v1 = val1.oneof_value
        length = len(_v1.signedIntValue)
        buff.write(_struct_I.pack(length))
        pattern = '<%sI'%length
        buff.write(struct.Struct(pattern).pack(*_v1.signedIntValue))
        length = len(_v1.unsignedIntValue)
        buff.write(_struct_I.pack(length))
        pattern = '<%sI'%length
        buff.write(struct.Struct(pattern).pack(*_v1.unsignedIntValue))
        length = len(_v1.floatValue)
        buff.write(_struct_I.pack(length))
        pattern = '<%sI'%length
        buff.write(struct.Struct(pattern).pack(*_v1.floatValue))
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
        self.input = kortex_driver.msg.Calibration()
      end = 0
      start = end
      end += 4
      (self.input.calibration_item,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.calibration_parameter = []
      for i in range(0, length):
        val1 = kortex_driver.msg.CalibrationParameter()
        start = end
        end += 4
        (val1.calibration_parameter_identifier,) = _get_struct_I().unpack(str[start:end])
        _v2 = val1.oneof_value
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sI'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        _v2.signedIntValue = s.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sI'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        _v2.unsignedIntValue = s.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sI'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        _v2.floatValue = s.unpack(str[start:end])
        self.input.calibration_parameter.append(val1)
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
      _x = self.input.calibration_item
      buff.write(_get_struct_I().pack(_x))
      length = len(self.input.calibration_parameter)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.calibration_parameter:
        _x = val1.calibration_parameter_identifier
        buff.write(_get_struct_I().pack(_x))
        _v3 = val1.oneof_value
        length = len(_v3.signedIntValue)
        buff.write(_struct_I.pack(length))
        pattern = '<%sI'%length
        buff.write(_v3.signedIntValue.tostring())
        length = len(_v3.unsignedIntValue)
        buff.write(_struct_I.pack(length))
        pattern = '<%sI'%length
        buff.write(_v3.unsignedIntValue.tostring())
        length = len(_v3.floatValue)
        buff.write(_struct_I.pack(length))
        pattern = '<%sI'%length
        buff.write(_v3.floatValue.tostring())
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
        self.input = kortex_driver.msg.Calibration()
      end = 0
      start = end
      end += 4
      (self.input.calibration_item,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.calibration_parameter = []
      for i in range(0, length):
        val1 = kortex_driver.msg.CalibrationParameter()
        start = end
        end += 4
        (val1.calibration_parameter_identifier,) = _get_struct_I().unpack(str[start:end])
        _v4 = val1.oneof_value
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sI'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        _v4.signedIntValue = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sI'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        _v4.unsignedIntValue = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sI'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        _v4.floatValue = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
        self.input.calibration_parameter.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/ExecuteCalibrationResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class ExecuteCalibrationResponse(genpy.Message):
  _md5sum = "c6c43d221c810050f75091660f63b0cd"
  _type = "kortex_driver/ExecuteCalibrationResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """Empty output

================================================================================
MSG: kortex_driver/Empty
"""
  __slots__ = ['output']
  _slot_types = ['kortex_driver/Empty']

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
      super(ExecuteCalibrationResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.output is None:
        self.output = kortex_driver.msg.Empty()
    else:
      self.output = kortex_driver.msg.Empty()

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
      pass
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
        self.output = kortex_driver.msg.Empty()
      end = 0
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
      pass
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
        self.output = kortex_driver.msg.Empty()
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
class ExecuteCalibration(object):
  _type          = 'kortex_driver/ExecuteCalibration'
  _md5sum = '8aabe5c823e6d1cd3d6eddd247265dae'
  _request_class  = ExecuteCalibrationRequest
  _response_class = ExecuteCalibrationResponse
