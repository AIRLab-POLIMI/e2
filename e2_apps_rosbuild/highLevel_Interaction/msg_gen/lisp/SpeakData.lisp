; Auto-generated. Do not edit!


(cl:in-package highLevel_Interaction-msg)


;//! \htmlinclude SpeakData.msg.html

(cl:defclass <SpeakData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (phraseNum
    :reader phraseNum
    :initarg :phraseNum
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32)))
)

(cl:defclass SpeakData (<SpeakData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeakData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeakData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name highLevel_Interaction-msg:<SpeakData> is deprecated: use highLevel_Interaction-msg:SpeakData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SpeakData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader highLevel_Interaction-msg:header-val is deprecated.  Use highLevel_Interaction-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'phraseNum-val :lambda-list '(m))
(cl:defmethod phraseNum-val ((m <SpeakData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader highLevel_Interaction-msg:phraseNum-val is deprecated.  Use highLevel_Interaction-msg:phraseNum instead.")
  (phraseNum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeakData>) ostream)
  "Serializes a message object of type '<SpeakData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'phraseNum) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeakData>) istream)
  "Deserializes a message object of type '<SpeakData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'phraseNum) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeakData>)))
  "Returns string type for a message object of type '<SpeakData>"
  "highLevel_Interaction/SpeakData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeakData)))
  "Returns string type for a message object of type 'SpeakData"
  "highLevel_Interaction/SpeakData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeakData>)))
  "Returns md5sum for a message object of type '<SpeakData>"
  "b6cff6f5fda3edef1242a7e3d5fe36e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeakData)))
  "Returns md5sum for a message object of type 'SpeakData"
  "b6cff6f5fda3edef1242a7e3d5fe36e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeakData>)))
  "Returns full string definition for message of type '<SpeakData>"
  (cl:format cl:nil "Header header~%std_msgs/Int32 phraseNum~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeakData)))
  "Returns full string definition for message of type 'SpeakData"
  (cl:format cl:nil "Header header~%std_msgs/Int32 phraseNum~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeakData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'phraseNum))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeakData>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeakData
    (cl:cons ':header (header msg))
    (cl:cons ':phraseNum (phraseNum msg))
))
