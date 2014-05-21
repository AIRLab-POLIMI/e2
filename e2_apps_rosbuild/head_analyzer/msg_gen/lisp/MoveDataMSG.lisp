; Auto-generated. Do not edit!


(cl:in-package head_analyzer-msg)


;//! \htmlinclude MoveDataMSG.msg.html

(cl:defclass <MoveDataMSG> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (moveClassification
    :reader moveClassification
    :initarg :moveClassification
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass MoveDataMSG (<MoveDataMSG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveDataMSG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveDataMSG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name head_analyzer-msg:<MoveDataMSG> is deprecated: use head_analyzer-msg:MoveDataMSG instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MoveDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:header-val is deprecated.  Use head_analyzer-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'moveClassification-val :lambda-list '(m))
(cl:defmethod moveClassification-val ((m <MoveDataMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader head_analyzer-msg:moveClassification-val is deprecated.  Use head_analyzer-msg:moveClassification instead.")
  (moveClassification m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveDataMSG>) ostream)
  "Serializes a message object of type '<MoveDataMSG>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'moveClassification) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveDataMSG>) istream)
  "Deserializes a message object of type '<MoveDataMSG>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'moveClassification) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveDataMSG>)))
  "Returns string type for a message object of type '<MoveDataMSG>"
  "head_analyzer/MoveDataMSG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveDataMSG)))
  "Returns string type for a message object of type 'MoveDataMSG"
  "head_analyzer/MoveDataMSG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveDataMSG>)))
  "Returns md5sum for a message object of type '<MoveDataMSG>"
  "e700bd3348d89cedd65fc9aed3932f1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveDataMSG)))
  "Returns md5sum for a message object of type 'MoveDataMSG"
  "e700bd3348d89cedd65fc9aed3932f1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveDataMSG>)))
  "Returns full string definition for message of type '<MoveDataMSG>"
  (cl:format cl:nil "Header header~%std_msgs/String moveClassification~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveDataMSG)))
  "Returns full string definition for message of type 'MoveDataMSG"
  (cl:format cl:nil "Header header~%std_msgs/String moveClassification~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveDataMSG>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'moveClassification))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveDataMSG>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveDataMSG
    (cl:cons ':header (header msg))
    (cl:cons ':moveClassification (moveClassification msg))
))
