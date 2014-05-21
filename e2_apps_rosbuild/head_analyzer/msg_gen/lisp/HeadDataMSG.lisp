; Auto-generated. Do not edit!


(cl:in-package head_analyzer-msg)


;//! \htmlinclude HeadDataMSG.msg.html

(cl:defclass <HeadDataMSG> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (headRatio
    :reader headRatio
    :initarg :headRatio
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (headPitch
    :reader headPitch
    :initarg :headPitch
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (headRoll
    :reader headRoll
    :initarg :headRoll
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32)))
)

(cl:defclass HeadDataMSG (<HeadDataMSG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HeadDataMSG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HeadDataMSG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name head_analyzer-msg:<HeadDataMSG> is deprecated: use head_analyzer-msg:HeadDataMSG instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HeadDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:header-val is deprecated.  Use head_analyzer-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'headRatio-val :lambda-list '(m))
(cl:defmethod headRatio-val ((m <HeadDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:headRatio-val is deprecated.  Use head_analyzer-msg:headRatio instead.")
  (headRatio m))

(cl:ensure-generic-function 'headPitch-val :lambda-list '(m))
(cl:defmethod headPitch-val ((m <HeadDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:headPitch-val is deprecated.  Use head_analyzer-msg:headPitch instead.")
  (headPitch m))

(cl:ensure-generic-function 'headRoll-val :lambda-list '(m))
(cl:defmethod headRoll-val ((m <HeadDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:headRoll-val is deprecated.  Use head_analyzer-msg:headRoll instead.")
  (headRoll m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HeadDataMSG>) ostream)
  "Serializes a message object of type '<HeadDataMSG>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'headRatio) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'headPitch) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'headRoll) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HeadDataMSG>) istream)
  "Deserializes a message object of type '<HeadDataMSG>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'headRatio) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'headPitch) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'headRoll) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HeadDataMSG>)))
  "Returns string type for a message object of type '<HeadDataMSG>"
  "head_analyzer/HeadDataMSG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HeadDataMSG)))
  "Returns string type for a message object of type 'HeadDataMSG"
  "head_analyzer/HeadDataMSG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HeadDataMSG>)))
  "Returns md5sum for a message object of type '<HeadDataMSG>"
  "aab940b9089e35044f140ef5c26adffd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HeadDataMSG)))
  "Returns md5sum for a message object of type 'HeadDataMSG"
  "aab940b9089e35044f140ef5c26adffd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HeadDataMSG>)))
  "Returns full string definition for message of type '<HeadDataMSG>"
  (cl:format cl:nil "Header header~%std_msgs/Float32 headRatio~%std_msgs/Int32 headPitch~%std_msgs/Int32 headRoll~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HeadDataMSG)))
  "Returns full string definition for message of type 'HeadDataMSG"
  (cl:format cl:nil "Header header~%std_msgs/Float32 headRatio~%std_msgs/Int32 headPitch~%std_msgs/Int32 headRoll~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HeadDataMSG>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'headRatio))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'headPitch))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'headRoll))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HeadDataMSG>))
  "Converts a ROS message object to a list"
  (cl:list 'HeadDataMSG
    (cl:cons ':header (header msg))
    (cl:cons ':headRatio (headRatio msg))
    (cl:cons ':headPitch (headPitch msg))
    (cl:cons ':headRoll (headRoll msg))
))
