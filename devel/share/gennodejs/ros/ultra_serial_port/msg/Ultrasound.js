// Auto-generated. Do not edit!

// (in-package ultra_serial_port.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Ultrasound {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dis = null;
      this.dis2 = null;
    }
    else {
      if (initObj.hasOwnProperty('dis')) {
        this.dis = initObj.dis
      }
      else {
        this.dis = 0;
      }
      if (initObj.hasOwnProperty('dis2')) {
        this.dis2 = initObj.dis2
      }
      else {
        this.dis2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Ultrasound
    // Serialize message field [dis]
    bufferOffset = _serializer.int16(obj.dis, buffer, bufferOffset);
    // Serialize message field [dis2]
    bufferOffset = _serializer.int16(obj.dis2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ultrasound
    let len;
    let data = new Ultrasound(null);
    // Deserialize message field [dis]
    data.dis = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dis2]
    data.dis2 = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ultra_serial_port/Ultrasound';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fbe7339e94fec895366dda899719a997';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 dis
    int16 dis2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Ultrasound(null);
    if (msg.dis !== undefined) {
      resolved.dis = msg.dis;
    }
    else {
      resolved.dis = 0
    }

    if (msg.dis2 !== undefined) {
      resolved.dis2 = msg.dis2;
    }
    else {
      resolved.dis2 = 0
    }

    return resolved;
    }
};

module.exports = Ultrasound;
