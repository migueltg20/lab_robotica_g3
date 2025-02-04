// Auto-generated. Do not edit!

// (in-package autonomous_navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Vector3Array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vectors = null;
    }
    else {
      if (initObj.hasOwnProperty('vectors')) {
        this.vectors = initObj.vectors
      }
      else {
        this.vectors = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Vector3Array
    // Serialize message field [vectors]
    // Serialize the length for message field [vectors]
    bufferOffset = _serializer.uint32(obj.vectors.length, buffer, bufferOffset);
    obj.vectors.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Vector3Array
    let len;
    let data = new Vector3Array(null);
    // Deserialize message field [vectors]
    // Deserialize array length for message field [vectors]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.vectors = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.vectors[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.vectors.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'autonomous_navigation/Vector3Array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e70d69b80b6619295db7fb48376314f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Vector3Array.msg
    # Un mensaje que contiene un arreglo de geometry_msgs/Vector3
    
    geometry_msgs/Vector3[] vectors
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Vector3Array(null);
    if (msg.vectors !== undefined) {
      resolved.vectors = new Array(msg.vectors.length);
      for (let i = 0; i < resolved.vectors.length; ++i) {
        resolved.vectors[i] = geometry_msgs.msg.Vector3.Resolve(msg.vectors[i]);
      }
    }
    else {
      resolved.vectors = []
    }

    return resolved;
    }
};

module.exports = Vector3Array;
