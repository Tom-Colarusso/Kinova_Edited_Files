# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/TrajectoryErrorType.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class TrajectoryErrorType(genpy.Message):
  _md5sum = "4f3aa449e7cdc70f504c12f27c350f66"
  _type = "kortex_driver/TrajectoryErrorType"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
uint32 TRAJECTORY_ERROR_TYPE_UNSPECIFIED = 0

uint32 TRAJECTORY_ERROR_TYPE_OUTSIDE_WORKSPACE = 1

uint32 TRAJECTORY_ERROR_TYPE_ACTUATOR_COUNT_MISMATCH = 2

uint32 TRAJECTORY_ERROR_TYPE_INVALID_DURATION = 3

uint32 TRAJECTORY_ERROR_TYPE_JOINT_NO_MOTION = 4

uint32 TRAJECTORY_ERROR_TYPE_ZERO_DISTANCE = 4

uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED = 5

uint32 TRAJECTORY_ERROR_TYPE_INVALID_SPEED = 5

uint32 TRAJECTORY_ERROR_TYPE_LARGE_SPEED = 6

uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_ACCELERATION = 7

uint32 TRAJECTORY_ERROR_TYPE_INVALID_ACCELERATION = 7

uint32 TRAJECTORY_ERROR_TYPE_INVALID_TIME_STEP = 8

uint32 TRAJECTORY_ERROR_TYPE_INVALID_TRAJECTORY_SIZE = 9

uint32 TRAJECTORY_ERROR_TYPE_LARGE_SIZE = 9

uint32 TRAJECTORY_ERROR_TYPE_WRONG_MODE = 10

uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_POSITION = 11

uint32 TRAJECTORY_ERROR_TYPE_JOINT_POSITION_LIMIT = 11

uint32 TRAJECTORY_ERROR_TYPE_FILE_ERROR = 12

uint32 TRAJECTORY_ERROR_TYPE_NO_FILE_IN_MEMORY = 13

uint32 TRAJECTORY_ERROR_TYPE_INDEX_OUT_OF_TRAJ = 14

uint32 TRAJECTORY_ERROR_TYPE_TRAJECTORY_ALREADY_RUNNING = 15

uint32 TRAJECTORY_ERROR_TYPE_ALREADY_RUNNING = 15

uint32 TRAJECTORY_ERROR_TYPE_WRONG_STARTING_POINT = 16

uint32 TRAJECTORY_ERROR_TYPE_CARTESIAN_CANNOT_START = 17

uint32 TRAJECTORY_ERROR_TYPE_WRONG_STARTING_SPEED = 18

uint32 TRAJECTORY_ERROR_TYPE_INVALID_POSITION = 19

uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_POSITION = 20

uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ORIENTATION = 21

uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_LINEAR_VELOCITY = 22

uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ANGULAR_VELOCITY = 23

uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_TORQUE = 24

uint32 TRAJECTORY_ERROR_TYPE_MULTIPLE_WAYPOINT_TYPE_LIST = 25

uint32 TRAJECTORY_ERROR_TYPE_INITIAL_WAYPOINT_NO_STOP = 26

uint32 TRAJECTORY_ERROR_TYPE_FINAL_WAYPOINT_NO_STOP = 27

uint32 TRAJECTORY_ERROR_TYPE_INVALID_BLENDING_RADIUS = 28

uint32 TRAJECTORY_ERROR_TYPE_INVALID_REFERENCE_FRAME = 29

uint32 TRAJECTORY_ERROR_TYPE_NUMERICAL_ERROR_IMPOSSIBLE_TRAJECTORY = 30
"""
  # Pseudo-constants
  TRAJECTORY_ERROR_TYPE_UNSPECIFIED = 0
  TRAJECTORY_ERROR_TYPE_OUTSIDE_WORKSPACE = 1
  TRAJECTORY_ERROR_TYPE_ACTUATOR_COUNT_MISMATCH = 2
  TRAJECTORY_ERROR_TYPE_INVALID_DURATION = 3
  TRAJECTORY_ERROR_TYPE_JOINT_NO_MOTION = 4
  TRAJECTORY_ERROR_TYPE_ZERO_DISTANCE = 4
  TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED = 5
  TRAJECTORY_ERROR_TYPE_INVALID_SPEED = 5
  TRAJECTORY_ERROR_TYPE_LARGE_SPEED = 6
  TRAJECTORY_ERROR_TYPE_INVALID_JOINT_ACCELERATION = 7
  TRAJECTORY_ERROR_TYPE_INVALID_ACCELERATION = 7
  TRAJECTORY_ERROR_TYPE_INVALID_TIME_STEP = 8
  TRAJECTORY_ERROR_TYPE_INVALID_TRAJECTORY_SIZE = 9
  TRAJECTORY_ERROR_TYPE_LARGE_SIZE = 9
  TRAJECTORY_ERROR_TYPE_WRONG_MODE = 10
  TRAJECTORY_ERROR_TYPE_INVALID_JOINT_POSITION = 11
  TRAJECTORY_ERROR_TYPE_JOINT_POSITION_LIMIT = 11
  TRAJECTORY_ERROR_TYPE_FILE_ERROR = 12
  TRAJECTORY_ERROR_TYPE_NO_FILE_IN_MEMORY = 13
  TRAJECTORY_ERROR_TYPE_INDEX_OUT_OF_TRAJ = 14
  TRAJECTORY_ERROR_TYPE_TRAJECTORY_ALREADY_RUNNING = 15
  TRAJECTORY_ERROR_TYPE_ALREADY_RUNNING = 15
  TRAJECTORY_ERROR_TYPE_WRONG_STARTING_POINT = 16
  TRAJECTORY_ERROR_TYPE_CARTESIAN_CANNOT_START = 17
  TRAJECTORY_ERROR_TYPE_WRONG_STARTING_SPEED = 18
  TRAJECTORY_ERROR_TYPE_INVALID_POSITION = 19
  TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_POSITION = 20
  TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ORIENTATION = 21
  TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_LINEAR_VELOCITY = 22
  TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ANGULAR_VELOCITY = 23
  TRAJECTORY_ERROR_TYPE_INVALID_JOINT_TORQUE = 24
  TRAJECTORY_ERROR_TYPE_MULTIPLE_WAYPOINT_TYPE_LIST = 25
  TRAJECTORY_ERROR_TYPE_INITIAL_WAYPOINT_NO_STOP = 26
  TRAJECTORY_ERROR_TYPE_FINAL_WAYPOINT_NO_STOP = 27
  TRAJECTORY_ERROR_TYPE_INVALID_BLENDING_RADIUS = 28
  TRAJECTORY_ERROR_TYPE_INVALID_REFERENCE_FRAME = 29
  TRAJECTORY_ERROR_TYPE_NUMERICAL_ERROR_IMPOSSIBLE_TRAJECTORY = 30

  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TrajectoryErrorType, self).__init__(*args, **kwds)

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
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
