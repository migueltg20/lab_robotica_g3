; Auto-generated. Do not edit!


(cl:in-package autonomous_navigation-msg)


;//! \htmlinclude Vector3Array.msg.html

(cl:defclass <Vector3Array> (roslisp-msg-protocol:ros-message)
  ((vectors
    :reader vectors
    :initarg :vectors
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3))))
)

(cl:defclass Vector3Array (<Vector3Array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vector3Array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vector3Array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autonomous_navigation-msg:<Vector3Array> is deprecated: use autonomous_navigation-msg:Vector3Array instead.")))

(cl:ensure-generic-function 'vectors-val :lambda-list '(m))
(cl:defmethod vectors-val ((m <Vector3Array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_navigation-msg:vectors-val is deprecated.  Use autonomous_navigation-msg:vectors instead.")
  (vectors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vector3Array>) ostream)
  "Serializes a message object of type '<Vector3Array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vectors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vectors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vector3Array>) istream)
  "Deserializes a message object of type '<Vector3Array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vectors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vectors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vector3Array>)))
  "Returns string type for a message object of type '<Vector3Array>"
  "autonomous_navigation/Vector3Array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vector3Array)))
  "Returns string type for a message object of type 'Vector3Array"
  "autonomous_navigation/Vector3Array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vector3Array>)))
  "Returns md5sum for a message object of type '<Vector3Array>"
  "0e70d69b80b6619295db7fb48376314f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vector3Array)))
  "Returns md5sum for a message object of type 'Vector3Array"
  "0e70d69b80b6619295db7fb48376314f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vector3Array>)))
  "Returns full string definition for message of type '<Vector3Array>"
  (cl:format cl:nil "# Vector3Array.msg~%# Un mensaje que contiene un arreglo de geometry_msgs/Vector3~%~%geometry_msgs/Vector3[] vectors~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vector3Array)))
  "Returns full string definition for message of type 'Vector3Array"
  (cl:format cl:nil "# Vector3Array.msg~%# Un mensaje que contiene un arreglo de geometry_msgs/Vector3~%~%geometry_msgs/Vector3[] vectors~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vector3Array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vectors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vector3Array>))
  "Converts a ROS message object to a list"
  (cl:list 'Vector3Array
    (cl:cons ':vectors (vectors msg))
))
