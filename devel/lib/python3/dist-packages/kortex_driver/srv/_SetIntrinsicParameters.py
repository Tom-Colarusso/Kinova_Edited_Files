# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/SetIntrinsicParametersRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class SetIntrinsicParametersRequest(genpy.Message):
  _md5sum = "5c2a2b6647655afd9c96bec51c351692"
  _type = "kortex_driver/SetIntrinsicParametersRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """IntrinsicParameters input

================================================================================
MSG: kortex_driver/IntrinsicParameters

uint32 sensor
uint32 resolution
float32 principal_point_x
float32 principal_point_y
float32 focal_length_x
float32 focal_length_y
DistortionCoefficients distortion_coeffs
================================================================================
MSG: kortex_driver/DistortionCoefficients

float32 k1
float32 k2
float32 k3
float32 p1
float32 p2"""
  __slots__ = ['input']
  _slot_types = ['kortex_driver/IntrinsicParameters']

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
      super(SetIntrinsicParametersRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.input is None:
        self.input = kortex_driver.msg.IntrinsicParameters()
    else:
      self.input = kortex_driver.msg.IntrinsicParameters()

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
      buff.write(_get_struct_2I9f().pack(_x.input.sensor, _x.input.resolution, _x.input.principal_point_x, _x.input.principal_point_y, _x.input.focal_length_x, _x.input.focal_length_y, _x.input.distortion_coeffs.k1, _x.input.distortion_coeffs.k2, _x.input.distortion_coeffs.k3, _x.input.distortion_coeffs.p1, _x.input.distortion_coeffs.p2))
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
        self.input = kortex_driver.msg.IntrinsicParameters()
      end = 0
      _x = self
      start = end
      end += 44
      (_x.input.sensor, _x.input.resolution, _x.input.principal_point_x, _x.input.principal_point_y, _x.input.focal_length_x, _x.input.focal_length_y, _x.input.distortion_coeffs.k1, _x.input.distortion_coeffs.k2, _x.input.distortion_coeffs.k3, _x.input.distortion_coeffs.p1, _x.input.distortion_coeffs.p2,) = _get_struct_2I9f().unpack(str[start:end])
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
      buff.write(_get_struct_2I9f().pack(_x.input.sensor, _x.input.resolution, _x.input.principal_point_x, _x.input.principal_point_y, _x.input.focal_length_x, _x.input.focal_length_y, _x.input.distortion_coeffs.k1, _x.input.distortion_coeffs.k2, _x.input.distortion_coeffs.k3, _x.input.distortion_coeffs.p1, _x.input.distortion_coeffs.p2))
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
        self.input = kortex_driver.msg.IntrinsicParameters()
      end = 0
      _x = self
      start = end
      end += 44
      (_x.input.sensor, _x.input.resolution, _x.input.principal_point_x, _x.input.principal_point_y, _x.input.focal_length_x, _x.input.focal_length_y, _x.input.distortion_coeffs.k1, _x.input.distortion_coeffs.k2, _x.input.distortion_coeffs.k3, _x.input.distortion_coeffs.p1, _x.input.distortion_coeffs.p2,) = _get_struct_2I9f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I9f = None
def _get_struct_2I9f():
    global _struct_2I9f
    if _struct_2I9f is None:
        _struct_2I9f = struct.Struct("<2I9f")
    return _struct_2I9f
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/SetIntrinsicParametersResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class SetIntrinsicParametersResponse(genpy.Message):
  _md5sum = "c6c43d221c810050f75091660f63b0cd"
  _type = "kortex_driver/SetIntrinsicParametersResponse"
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
      super(SetIntrinsicParametersResponse, self).__init__(*args, **kwds)
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
class SetIntrinsicParameters(object):
  _type          = 'kortex_driver/SetIntrinsicParameters'
  _md5sum = 'e4fa2fa37ca4b5105af9a0b157505e8c'
  _request_class  = SetIntrinsicParametersRequest
  _response_class = SetIntrinsicParametersResponse
