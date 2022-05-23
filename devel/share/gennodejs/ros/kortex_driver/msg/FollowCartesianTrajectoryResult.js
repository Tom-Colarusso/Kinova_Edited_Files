// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FollowCartesianTrajectoryResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.error_string = null;
      this.error_code = null;
    }
    else {
      if (initObj.hasOwnProperty('error_string')) {
        this.error_string = initObj.error_string
      }
      else {
        this.error_string = '';
      }
      if (initObj.hasOwnProperty('error_code')) {
        this.error_code = initObj.error_code
      }
      else {
        this.error_code = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FollowCartesianTrajectoryResult
    // Serialize message field [error_string]
    bufferOffset = _serializer.string(obj.error_string, buffer, bufferOffset);
    // Serialize message field [error_code]
    bufferOffset = _serializer.int32(obj.error_code, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FollowCartesianTrajectoryResult
    let len;
    let data = new FollowCartesianTrajectoryResult(null);
    // Deserialize message field [error_string]
    data.error_string = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [error_code]
    data.error_code = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.error_string);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/FollowCartesianTrajectoryResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e845eb53c49e5687cc804e4383078872';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #result definition
    string error_string
    int32 error_code
    int32 SUCCESSFUL = 0
    int32 INVALID_GOAL = -1
    int32 PATH_TOLERANCE_VIOLATED = -2
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FollowCartesianTrajectoryResult(null);
    if (msg.error_string !== undefined) {
      resolved.error_string = msg.error_string;
    }
    else {
      resolved.error_string = ''
    }

    if (msg.error_code !== undefined) {
      resolved.error_code = msg.error_code;
    }
    else {
      resolved.error_code = 0
    }

    return resolved;
    }
};

// Constants for message
FollowCartesianTrajectoryResult.Constants = {
  SUCCESSFUL: 0,
  INVALID_GOAL: -1,
  PATH_TOLERANCE_VIOLATED: -2,
}

module.exports = FollowCartesianTrajectoryResult;
