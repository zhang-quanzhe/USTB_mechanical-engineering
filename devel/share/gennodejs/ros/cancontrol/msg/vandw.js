// Auto-generated. Do not edit!

// (in-package cancontrol.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class vandw {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.v = null;
      this.w = null;
      this.dis_x = null;
      this.dis_y = null;
    }
    else {
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0;
      }
      if (initObj.hasOwnProperty('w')) {
        this.w = initObj.w
      }
      else {
        this.w = 0;
      }
      if (initObj.hasOwnProperty('dis_x')) {
        this.dis_x = initObj.dis_x
      }
      else {
        this.dis_x = 0;
      }
      if (initObj.hasOwnProperty('dis_y')) {
        this.dis_y = initObj.dis_y
      }
      else {
        this.dis_y = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vandw
    // Serialize message field [v]
    bufferOffset = _serializer.int32(obj.v, buffer, bufferOffset);
    // Serialize message field [w]
    bufferOffset = _serializer.int32(obj.w, buffer, bufferOffset);
    // Serialize message field [dis_x]
    bufferOffset = _serializer.int32(obj.dis_x, buffer, bufferOffset);
    // Serialize message field [dis_y]
    bufferOffset = _serializer.int32(obj.dis_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vandw
    let len;
    let data = new vandw(null);
    // Deserialize message field [v]
    data.v = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [w]
    data.w = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [dis_x]
    data.dis_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [dis_y]
    data.dis_y = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cancontrol/vandw';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7fadd1e689290e03c5298e42f0e94d6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 v
    int32 w
    int32 dis_x
    int32 dis_y
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vandw(null);
    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0
    }

    if (msg.w !== undefined) {
      resolved.w = msg.w;
    }
    else {
      resolved.w = 0
    }

    if (msg.dis_x !== undefined) {
      resolved.dis_x = msg.dis_x;
    }
    else {
      resolved.dis_x = 0
    }

    if (msg.dis_y !== undefined) {
      resolved.dis_y = msg.dis_y;
    }
    else {
      resolved.dis_y = 0
    }

    return resolved;
    }
};

module.exports = vandw;
