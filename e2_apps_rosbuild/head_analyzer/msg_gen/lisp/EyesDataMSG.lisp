; Auto-generated. Do not edit!


(cl:in-package head_analyzer-msg)


;//! \htmlinclude EyesDataMSG.msg.html

(cl:defclass <EyesDataMSG> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (eyeLRatio
    :reader eyeLRatio
    :initarg :eyeLRatio
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (eyeRRatio
    :reader eyeRRatio
    :initarg :eyeRRatio
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass EyesDataMSG (<EyesDataMSG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EyesDataMSG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EyesDataMSG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name head_analyzer-msg:<EyesDataMSG> is deprecated: use head_analyzer-msg:EyesDataMSG instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EyesDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:header-val is deprecated.  Use head_analyzer-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'eyeLRatio-val :lambda-list '(m))
(cl:defmethod eyeLRatio-val ((m <EyesDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:eyeLRatio-val is deprecated.  Use head_analyzer-msg:eyeLRatio instead.")
  (eyeLRatio m))

(cl:ensure-generic-function 'eyeRRatio-val :lambda-list '(m))
(cl:defmethod eyeRRatio-val ((m <EyesDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:eyeRRatio-val is deprecated.  Use head_analyzer-msg:eyeRRatio instead.")
  (eyeRRatio m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EyesDataMSG>) ostream)
  "Serializes a message object of type '<EyesDataMSG>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'eyeLRatio) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'eyeRRatio) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EyesDataMSG>) istream)
  "Deserializes a message object of type '<EyesDataMSG>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'eyeLRatio) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'eyeRRatio) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EyesDataMSG>)))
  "Returns string type for a message object of type '<EyesDataMSG>"
  "head_analyzer/EyesDataMSG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EyesDataMSG)))
  "Returns string type for a message object of type 'EyesDataMSG"
  "head_analyzer/EyesDataMSG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EyesDataMSG>)))
  "Returns md5sum for a message object of type '<EyesDataMSG>"
  "d40d8e3bb783a1495f8523b7fbc02d58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EyesDataMSG)))
  "Returns md5sum for a message object of type 'EyesDataMSG"
  "d40d8e3bb783a1495f8523b7fbc02d58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EyesDataMSG>)))
  "Returns full string definition for message of type '<EyesDataMSG>"
  (cl:format cl:nil "Header header~%std_msgs/Float32 eyeLRatio~%std_msgs/Float32 eyeRRatio~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EyesDataMSG)))
  "Returns full string definition for message of type 'EyesDataMSG"
  (cl:format cl:nil "Header header~%std_msgs/Float32 eyeLRatio~%std_msgs/Float32 eyeRRatio~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EyesDataMSG>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'eyeLRatio))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'eyeRRatio))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EyesDataMSG>))
  "Converts a ROS message object to a list"
  (cl:list 'EyesDataMSG
    (cl:cons ':header (header msg))
    (cl:cons ':eyeLRatio (eyeLRatio msg))
    (cl:cons ':eyeRRatio (eyeRRatio msg))
))
