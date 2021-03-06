# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/ActuatorCyclic_Feedback.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class ActuatorCyclic_Feedback(genpy.Message):
  _md5sum = "299d9bdfeb28700b38cf7f19f730d6c7"
  _type = "kortex_driver/ActuatorCyclic_Feedback"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
ActuatorCyclic_MessageId feedback_id
uint32 status_flags
uint32 jitter_comm
float32 position
float32 velocity
float32 torque
float32 current_motor
float32 voltage
float32 temperature_motor
float32 temperature_core
uint32 fault_bank_a
uint32 fault_bank_b
uint32 warning_bank_a
uint32 warning_bank_b
================================================================================
MSG: kortex_driver/ActuatorCyclic_MessageId

uint32 identifier"""
  __slots__ = ['feedback_id','status_flags','jitter_comm','position','velocity','torque','current_motor','voltage','temperature_motor','temperature_core','fault_bank_a','fault_bank_b','warning_bank_a','warning_bank_b']
  _slot_types = ['kortex_driver/ActuatorCyclic_MessageId','uint32','uint32','float32','float32','float32','float32','float32','float32','float32','uint32','uint32','uint32','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       feedback_id,status_flags,jitter_comm,position,velocity,torque,current_motor,voltage,temperature_motor,temperature_core,fault_bank_a,fault_bank_b,warning_bank_a,warning_bank_b

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ActuatorCyclic_Feedback, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.feedback_id is None:
        self.feedback_id = kortex_driver.msg.ActuatorCyclic_MessageId()
      if self.status_flags is None:
        self.status_flags = 0
      if self.jitter_comm is None:
        self.jitter_comm = 0
      if self.position is None:
        self.position = 0.
      if self.velocity is None:
        self.velocity = 0.
      if self.torque is None:
        self.torque = 0.
      if self.current_motor is None:
        self.current_motor = 0.
      if self.voltage is None:
        self.voltage = 0.
      if self.temperature_motor is None:
        self.temperature_motor = 0.
      if self.temperature_core is None:
        self.temperature_core = 0.
      if self.fault_bank_a is None:
        self.fault_bank_a = 0
      if self.fault_bank_b is None:
        self.fault_bank_b = 0
      if self.warning_bank_a is None:
        self.warning_bank_a = 0
      if self.warning_bank_b is None:
        self.warning_bank_b = 0
    else:
      self.feedback_id = kortex_driver.msg.ActuatorCyclic_MessageId()
      self.status_flags = 0
      self.jitter_comm = 0
      self.position = 0.
      self.velocity = 0.
      self.torque = 0.
      self.current_motor = 0.
      self.voltage = 0.
      self.temperature_motor = 0.
      self.temperature_core = 0.
      self.fault_bank_a = 0
      self.fault_bank_b = 0
      self.warning_bank_a = 0
      self.warning_bank_b = 0

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
      buff.write(_get_struct_3I7f4I().pack(_x.feedback_id.identifier, _x.status_flags, _x.jitter_comm, _x.position, _x.velocity, _x.torque, _x.current_motor, _x.voltage, _x.temperature_motor, _x.temperature_core, _x.fault_bank_a, _x.fault_bank_b, _x.warning_bank_a, _x.warning_bank_b))
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
      if self.feedback_id is None:
        self.feedback_id = kortex_driver.msg.ActuatorCyclic_MessageId()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.feedback_id.identifier, _x.status_flags, _x.jitter_comm, _x.position, _x.velocity, _x.torque, _x.current_motor, _x.voltage, _x.temperature_motor, _x.temperature_core, _x.fault_bank_a, _x.fault_bank_b, _x.warning_bank_a, _x.warning_bank_b,) = _get_struct_3I7f4I().unpack(str[start:end])
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
      buff.write(_get_struct_3I7f4I().pack(_x.feedback_id.identifier, _x.status_flags, _x.jitter_comm, _x.position, _x.velocity, _x.torque, _x.current_motor, _x.voltage, _x.temperature_motor, _x.temperature_core, _x.fault_bank_a, _x.fault_bank_b, _x.warning_bank_a, _x.warning_bank_b))
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
      if self.feedback_id is None:
        self.feedback_id = kortex_driver.msg.ActuatorCyclic_MessageId()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.feedback_id.identifier, _x.status_flags, _x.jitter_comm, _x.position, _x.velocity, _x.torque, _x.current_motor, _x.voltage, _x.temperature_motor, _x.temperature_core, _x.fault_bank_a, _x.fault_bank_b, _x.warning_bank_a, _x.warning_bank_b,) = _get_struct_3I7f4I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I7f4I = None
def _get_struct_3I7f4I():
    global _struct_3I7f4I
    if _struct_3I7f4I is None:
        _struct_3I7f4I = struct.Struct("<3I7f4I")
    return _struct_3I7f4I
