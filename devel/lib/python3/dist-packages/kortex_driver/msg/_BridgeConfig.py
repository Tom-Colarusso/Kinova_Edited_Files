# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/BridgeConfig.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class BridgeConfig(genpy.Message):
  _md5sum = "967c7a78caf96ff069310456674faf6a"
  _type = "kortex_driver/BridgeConfig"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
uint32 device_identifier
uint32 bridgetype
BridgePortConfig port_config
BridgeIdentifier bridge_id
================================================================================
MSG: kortex_driver/BridgePortConfig

uint32 target_port
uint32 out_port
================================================================================
MSG: kortex_driver/BridgeIdentifier

uint32 bridge_id"""
  __slots__ = ['device_identifier','bridgetype','port_config','bridge_id']
  _slot_types = ['uint32','uint32','kortex_driver/BridgePortConfig','kortex_driver/BridgeIdentifier']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       device_identifier,bridgetype,port_config,bridge_id

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BridgeConfig, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.device_identifier is None:
        self.device_identifier = 0
      if self.bridgetype is None:
        self.bridgetype = 0
      if self.port_config is None:
        self.port_config = kortex_driver.msg.BridgePortConfig()
      if self.bridge_id is None:
        self.bridge_id = kortex_driver.msg.BridgeIdentifier()
    else:
      self.device_identifier = 0
      self.bridgetype = 0
      self.port_config = kortex_driver.msg.BridgePortConfig()
      self.bridge_id = kortex_driver.msg.BridgeIdentifier()

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
      buff.write(_get_struct_5I().pack(_x.device_identifier, _x.bridgetype, _x.port_config.target_port, _x.port_config.out_port, _x.bridge_id.bridge_id))
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
      if self.port_config is None:
        self.port_config = kortex_driver.msg.BridgePortConfig()
      if self.bridge_id is None:
        self.bridge_id = kortex_driver.msg.BridgeIdentifier()
      end = 0
      _x = self
      start = end
      end += 20
      (_x.device_identifier, _x.bridgetype, _x.port_config.target_port, _x.port_config.out_port, _x.bridge_id.bridge_id,) = _get_struct_5I().unpack(str[start:end])
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
      buff.write(_get_struct_5I().pack(_x.device_identifier, _x.bridgetype, _x.port_config.target_port, _x.port_config.out_port, _x.bridge_id.bridge_id))
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
      if self.port_config is None:
        self.port_config = kortex_driver.msg.BridgePortConfig()
      if self.bridge_id is None:
        self.bridge_id = kortex_driver.msg.BridgeIdentifier()
      end = 0
      _x = self
      start = end
      end += 20
      (_x.device_identifier, _x.bridgetype, _x.port_config.target_port, _x.port_config.out_port, _x.bridge_id.bridge_id,) = _get_struct_5I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5I = None
def _get_struct_5I():
    global _struct_5I
    if _struct_5I is None:
        _struct_5I = struct.Struct("<5I")
    return _struct_5I
