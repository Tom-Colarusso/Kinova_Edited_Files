; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TrajectoryErrorType.msg.html

(cl:defclass <TrajectoryErrorType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TrajectoryErrorType (<TrajectoryErrorType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryErrorType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryErrorType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TrajectoryErrorType> is deprecated: use kortex_driver-msg:TrajectoryErrorType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryErrorType>)))
    "Constants for message type '<TrajectoryErrorType>"
  '((:TRAJECTORY_ERROR_TYPE_UNSPECIFIED . 0)
    (:TRAJECTORY_ERROR_TYPE_OUTSIDE_WORKSPACE . 1)
    (:TRAJECTORY_ERROR_TYPE_ACTUATOR_COUNT_MISMATCH . 2)
    (:TRAJECTORY_ERROR_TYPE_INVALID_DURATION . 3)
    (:TRAJECTORY_ERROR_TYPE_JOINT_NO_MOTION . 4)
    (:TRAJECTORY_ERROR_TYPE_ZERO_DISTANCE . 4)
    (:TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED . 5)
    (:TRAJECTORY_ERROR_TYPE_INVALID_SPEED . 5)
    (:TRAJECTORY_ERROR_TYPE_LARGE_SPEED . 6)
    (:TRAJECTORY_ERROR_TYPE_INVALID_JOINT_ACCELERATION . 7)
    (:TRAJECTORY_ERROR_TYPE_INVALID_ACCELERATION . 7)
    (:TRAJECTORY_ERROR_TYPE_INVALID_TIME_STEP . 8)
    (:TRAJECTORY_ERROR_TYPE_INVALID_TRAJECTORY_SIZE . 9)
    (:TRAJECTORY_ERROR_TYPE_LARGE_SIZE . 9)
    (:TRAJECTORY_ERROR_TYPE_WRONG_MODE . 10)
    (:TRAJECTORY_ERROR_TYPE_INVALID_JOINT_POSITION . 11)
    (:TRAJECTORY_ERROR_TYPE_JOINT_POSITION_LIMIT . 11)
    (:TRAJECTORY_ERROR_TYPE_FILE_ERROR . 12)
    (:TRAJECTORY_ERROR_TYPE_NO_FILE_IN_MEMORY . 13)
    (:TRAJECTORY_ERROR_TYPE_INDEX_OUT_OF_TRAJ . 14)
    (:TRAJECTORY_ERROR_TYPE_TRAJECTORY_ALREADY_RUNNING . 15)
    (:TRAJECTORY_ERROR_TYPE_ALREADY_RUNNING . 15)
    (:TRAJECTORY_ERROR_TYPE_WRONG_STARTING_POINT . 16)
    (:TRAJECTORY_ERROR_TYPE_CARTESIAN_CANNOT_START . 17)
    (:TRAJECTORY_ERROR_TYPE_WRONG_STARTING_SPEED . 18)
    (:TRAJECTORY_ERROR_TYPE_INVALID_POSITION . 19)
    (:TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_POSITION . 20)
    (:TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ORIENTATION . 21)
    (:TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_LINEAR_VELOCITY . 22)
    (:TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ANGULAR_VELOCITY . 23)
    (:TRAJECTORY_ERROR_TYPE_INVALID_JOINT_TORQUE . 24)
    (:TRAJECTORY_ERROR_TYPE_MULTIPLE_WAYPOINT_TYPE_LIST . 25)
    (:TRAJECTORY_ERROR_TYPE_INITIAL_WAYPOINT_NO_STOP . 26)
    (:TRAJECTORY_ERROR_TYPE_FINAL_WAYPOINT_NO_STOP . 27)
    (:TRAJECTORY_ERROR_TYPE_INVALID_BLENDING_RADIUS . 28)
    (:TRAJECTORY_ERROR_TYPE_INVALID_REFERENCE_FRAME . 29)
    (:TRAJECTORY_ERROR_TYPE_NUMERICAL_ERROR_IMPOSSIBLE_TRAJECTORY . 30))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryErrorType)))
    "Constants for message type 'TrajectoryErrorType"
  '((:TRAJECTORY_ERROR_TYPE_UNSPECIFIED . 0)
    (:TRAJECTORY_ERROR_TYPE_OUTSIDE_WORKSPACE . 1)
    (:TRAJECTORY_ERROR_TYPE_ACTUATOR_COUNT_MISMATCH . 2)
    (:TRAJECTORY_ERROR_TYPE_INVALID_DURATION . 3)
    (:TRAJECTORY_ERROR_TYPE_JOINT_NO_MOTION . 4)
    (:TRAJECTORY_ERROR_TYPE_ZERO_DISTANCE . 4)
    (:TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED . 5)
    (:TRAJECTORY_ERROR_TYPE_INVALID_SPEED . 5)
    (:TRAJECTORY_ERROR_TYPE_LARGE_SPEED . 6)
    (:TRAJECTORY_ERROR_TYPE_INVALID_JOINT_ACCELERATION . 7)
    (:TRAJECTORY_ERROR_TYPE_INVALID_ACCELERATION . 7)
    (:TRAJECTORY_ERROR_TYPE_INVALID_TIME_STEP . 8)
    (:TRAJECTORY_ERROR_TYPE_INVALID_TRAJECTORY_SIZE . 9)
    (:TRAJECTORY_ERROR_TYPE_LARGE_SIZE . 9)
    (:TRAJECTORY_ERROR_TYPE_WRONG_MODE . 10)
    (:TRAJECTORY_ERROR_TYPE_INVALID_JOINT_POSITION . 11)
    (:TRAJECTORY_ERROR_TYPE_JOINT_POSITION_LIMIT . 11)
    (:TRAJECTORY_ERROR_TYPE_FILE_ERROR . 12)
    (:TRAJECTORY_ERROR_TYPE_NO_FILE_IN_MEMORY . 13)
    (:TRAJECTORY_ERROR_TYPE_INDEX_OUT_OF_TRAJ . 14)
    (:TRAJECTORY_ERROR_TYPE_TRAJECTORY_ALREADY_RUNNING . 15)
    (:TRAJECTORY_ERROR_TYPE_ALREADY_RUNNING . 15)
    (:TRAJECTORY_ERROR_TYPE_WRONG_STARTING_POINT . 16)
    (:TRAJECTORY_ERROR_TYPE_CARTESIAN_CANNOT_START . 17)
    (:TRAJECTORY_ERROR_TYPE_WRONG_STARTING_SPEED . 18)
    (:TRAJECTORY_ERROR_TYPE_INVALID_POSITION . 19)
    (:TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_POSITION . 20)
    (:TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ORIENTATION . 21)
    (:TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_LINEAR_VELOCITY . 22)
    (:TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ANGULAR_VELOCITY . 23)
    (:TRAJECTORY_ERROR_TYPE_INVALID_JOINT_TORQUE . 24)
    (:TRAJECTORY_ERROR_TYPE_MULTIPLE_WAYPOINT_TYPE_LIST . 25)
    (:TRAJECTORY_ERROR_TYPE_INITIAL_WAYPOINT_NO_STOP . 26)
    (:TRAJECTORY_ERROR_TYPE_FINAL_WAYPOINT_NO_STOP . 27)
    (:TRAJECTORY_ERROR_TYPE_INVALID_BLENDING_RADIUS . 28)
    (:TRAJECTORY_ERROR_TYPE_INVALID_REFERENCE_FRAME . 29)
    (:TRAJECTORY_ERROR_TYPE_NUMERICAL_ERROR_IMPOSSIBLE_TRAJECTORY . 30))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryErrorType>) ostream)
  "Serializes a message object of type '<TrajectoryErrorType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryErrorType>) istream)
  "Deserializes a message object of type '<TrajectoryErrorType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryErrorType>)))
  "Returns string type for a message object of type '<TrajectoryErrorType>"
  "kortex_driver/TrajectoryErrorType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryErrorType)))
  "Returns string type for a message object of type 'TrajectoryErrorType"
  "kortex_driver/TrajectoryErrorType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryErrorType>)))
  "Returns md5sum for a message object of type '<TrajectoryErrorType>"
  "4f3aa449e7cdc70f504c12f27c350f66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryErrorType)))
  "Returns md5sum for a message object of type 'TrajectoryErrorType"
  "4f3aa449e7cdc70f504c12f27c350f66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryErrorType>)))
  "Returns full string definition for message of type '<TrajectoryErrorType>"
  (cl:format cl:nil "~%uint32 TRAJECTORY_ERROR_TYPE_UNSPECIFIED = 0~%~%uint32 TRAJECTORY_ERROR_TYPE_OUTSIDE_WORKSPACE = 1~%~%uint32 TRAJECTORY_ERROR_TYPE_ACTUATOR_COUNT_MISMATCH = 2~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_DURATION = 3~%~%uint32 TRAJECTORY_ERROR_TYPE_JOINT_NO_MOTION = 4~%~%uint32 TRAJECTORY_ERROR_TYPE_ZERO_DISTANCE = 4~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED = 5~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_SPEED = 5~%~%uint32 TRAJECTORY_ERROR_TYPE_LARGE_SPEED = 6~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_ACCELERATION = 7~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_ACCELERATION = 7~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_TIME_STEP = 8~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_TRAJECTORY_SIZE = 9~%~%uint32 TRAJECTORY_ERROR_TYPE_LARGE_SIZE = 9~%~%uint32 TRAJECTORY_ERROR_TYPE_WRONG_MODE = 10~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_POSITION = 11~%~%uint32 TRAJECTORY_ERROR_TYPE_JOINT_POSITION_LIMIT = 11~%~%uint32 TRAJECTORY_ERROR_TYPE_FILE_ERROR = 12~%~%uint32 TRAJECTORY_ERROR_TYPE_NO_FILE_IN_MEMORY = 13~%~%uint32 TRAJECTORY_ERROR_TYPE_INDEX_OUT_OF_TRAJ = 14~%~%uint32 TRAJECTORY_ERROR_TYPE_TRAJECTORY_ALREADY_RUNNING = 15~%~%uint32 TRAJECTORY_ERROR_TYPE_ALREADY_RUNNING = 15~%~%uint32 TRAJECTORY_ERROR_TYPE_WRONG_STARTING_POINT = 16~%~%uint32 TRAJECTORY_ERROR_TYPE_CARTESIAN_CANNOT_START = 17~%~%uint32 TRAJECTORY_ERROR_TYPE_WRONG_STARTING_SPEED = 18~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_POSITION = 19~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_POSITION = 20~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ORIENTATION = 21~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_LINEAR_VELOCITY = 22~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ANGULAR_VELOCITY = 23~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_TORQUE = 24~%~%uint32 TRAJECTORY_ERROR_TYPE_MULTIPLE_WAYPOINT_TYPE_LIST = 25~%~%uint32 TRAJECTORY_ERROR_TYPE_INITIAL_WAYPOINT_NO_STOP = 26~%~%uint32 TRAJECTORY_ERROR_TYPE_FINAL_WAYPOINT_NO_STOP = 27~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_BLENDING_RADIUS = 28~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_REFERENCE_FRAME = 29~%~%uint32 TRAJECTORY_ERROR_TYPE_NUMERICAL_ERROR_IMPOSSIBLE_TRAJECTORY = 30~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryErrorType)))
  "Returns full string definition for message of type 'TrajectoryErrorType"
  (cl:format cl:nil "~%uint32 TRAJECTORY_ERROR_TYPE_UNSPECIFIED = 0~%~%uint32 TRAJECTORY_ERROR_TYPE_OUTSIDE_WORKSPACE = 1~%~%uint32 TRAJECTORY_ERROR_TYPE_ACTUATOR_COUNT_MISMATCH = 2~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_DURATION = 3~%~%uint32 TRAJECTORY_ERROR_TYPE_JOINT_NO_MOTION = 4~%~%uint32 TRAJECTORY_ERROR_TYPE_ZERO_DISTANCE = 4~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED = 5~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_SPEED = 5~%~%uint32 TRAJECTORY_ERROR_TYPE_LARGE_SPEED = 6~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_ACCELERATION = 7~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_ACCELERATION = 7~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_TIME_STEP = 8~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_TRAJECTORY_SIZE = 9~%~%uint32 TRAJECTORY_ERROR_TYPE_LARGE_SIZE = 9~%~%uint32 TRAJECTORY_ERROR_TYPE_WRONG_MODE = 10~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_POSITION = 11~%~%uint32 TRAJECTORY_ERROR_TYPE_JOINT_POSITION_LIMIT = 11~%~%uint32 TRAJECTORY_ERROR_TYPE_FILE_ERROR = 12~%~%uint32 TRAJECTORY_ERROR_TYPE_NO_FILE_IN_MEMORY = 13~%~%uint32 TRAJECTORY_ERROR_TYPE_INDEX_OUT_OF_TRAJ = 14~%~%uint32 TRAJECTORY_ERROR_TYPE_TRAJECTORY_ALREADY_RUNNING = 15~%~%uint32 TRAJECTORY_ERROR_TYPE_ALREADY_RUNNING = 15~%~%uint32 TRAJECTORY_ERROR_TYPE_WRONG_STARTING_POINT = 16~%~%uint32 TRAJECTORY_ERROR_TYPE_CARTESIAN_CANNOT_START = 17~%~%uint32 TRAJECTORY_ERROR_TYPE_WRONG_STARTING_SPEED = 18~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_POSITION = 19~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_POSITION = 20~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ORIENTATION = 21~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_LINEAR_VELOCITY = 22~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_CARTESIAN_ANGULAR_VELOCITY = 23~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_JOINT_TORQUE = 24~%~%uint32 TRAJECTORY_ERROR_TYPE_MULTIPLE_WAYPOINT_TYPE_LIST = 25~%~%uint32 TRAJECTORY_ERROR_TYPE_INITIAL_WAYPOINT_NO_STOP = 26~%~%uint32 TRAJECTORY_ERROR_TYPE_FINAL_WAYPOINT_NO_STOP = 27~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_BLENDING_RADIUS = 28~%~%uint32 TRAJECTORY_ERROR_TYPE_INVALID_REFERENCE_FRAME = 29~%~%uint32 TRAJECTORY_ERROR_TYPE_NUMERICAL_ERROR_IMPOSSIBLE_TRAJECTORY = 30~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryErrorType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryErrorType>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryErrorType
))
