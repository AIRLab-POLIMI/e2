; Auto-generated. Do not edit!


(cl:in-package robot_brain-msg)


;//! \htmlinclude HighLevelData.msg.html

(cl:defclass <HighLevelData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (primitives
    :reader primitives
    :initarg :primitives
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (parameters
    :reader parameters
    :initarg :parameters
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass HighLevelData (<HighLevelData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HighLevelData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HighLevelData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_brain-msg:<HighLevelData> is deprecated: use robot_brain-msg:HighLevelData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HighLevelData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_brain-msg:header-val is deprecated.  Use robot_brain-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'primitives-val :lambda-list '(m))
(cl:defmethod primitives-val ((m <HighLevelData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_brain-msg:primitives-val is deprecated.  Use robot_brain-msg:primitives instead.")
  (primitives m))

(cl:ensure-generic-function 'parameters-val :lambda-list '(m))
(cl:defmethod parameters-val ((m <HighLevelData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_brain-msg:parameters-val is deprecated.  Use robot_brain-msg:parameters instead.")
  (parameters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HighLevelData>) ostream)
  "Serializes a message object of type '<HighLevelData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'primitives) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'parameters) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HighLevelData>) istream)
  "Deserializes a message object of type '<HighLevelData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'primitives) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'parameters) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HighLevelData>)))
  "Returns string type for a message object of type '<HighLevelData>"
  "robot_brain/HighLevelData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HighLevelData)))
  "Returns string type for a message object of type 'HighLevelData"
  "robot_brain/HighLevelData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HighLevelData>)))
  "Returns md5sum for a message object of type '<HighLevelData>"
  "18f2070099471189acbf4f0a469c7131")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HighLevelData)))
  "Returns md5sum for a message object of type 'HighLevelData"
  "18f2070099471189acbf4f0a469c7131")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HighLevelData>)))
  "Returns full string definition for message of type '<HighLevelData>"
  (cl:format cl:nil "Header header~%std_msgs/String primitives~%std_msgs/String parameters~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HighLevelData)))
  "Returns full string definition for message of type 'HighLevelData"
  (cl:format cl:nil "Header header~%std_msgs/String primitives~%std_msgs/String parameters~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HighLevelData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'primitives))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'parameters))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HighLevelData>))
  "Converts a ROS message object to a list"
  (cl:list 'HighLevelData
    (cl:cons ':header (header msg))
    (cl:cons ':primitives (primitives msg))
    (cl:cons ':parameters (parameters msg))
))
