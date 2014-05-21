; Auto-generated. Do not edit!


(cl:in-package robot_brain-msg)


;//! \htmlinclude WheelData.msg.html

(cl:defclass <WheelData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tanSpeed
    :reader tanSpeed
    :initarg :tanSpeed
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (rotSpeed
    :reader rotSpeed
    :initarg :rotSpeed
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32)))
)

(cl:defclass WheelData (<WheelData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_brain-msg:<WheelData> is deprecated: use robot_brain-msg:WheelData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WheelData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_brain-msg:header-val is deprecated.  Use robot_brain-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tanSpeed-val :lambda-list '(m))
(cl:defmethod tanSpeed-val ((m <WheelData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_brain-msg:tanSpeed-val is deprecated.  Use robot_brain-msg:tanSpeed instead.")
  (tanSpeed m))

(cl:ensure-generic-function 'rotSpeed-val :lambda-list '(m))
(cl:defmethod rotSpeed-val ((m <WheelData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_brain-msg:rotSpeed-val is deprecated.  Use robot_brain-msg:rotSpeed instead.")
  (rotSpeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelData>) ostream)
  "Serializes a message object of type '<WheelData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tanSpeed) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rotSpeed) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelData>) istream)
  "Deserializes a message object of type '<WheelData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tanSpeed) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rotSpeed) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelData>)))
  "Returns string type for a message object of type '<WheelData>"
  "robot_brain/WheelData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelData)))
  "Returns string type for a message object of type 'WheelData"
  "robot_brain/WheelData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelData>)))
  "Returns md5sum for a message object of type '<WheelData>"
  "e99e8207d3c6ebdd64952205c3e902cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelData)))
  "Returns md5sum for a message object of type 'WheelData"
  "e99e8207d3c6ebdd64952205c3e902cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelData>)))
  "Returns full string definition for message of type '<WheelData>"
  (cl:format cl:nil "Header header~%std_msgs/Int32 tanSpeed~%std_msgs/Int32 rotSpeed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelData)))
  "Returns full string definition for message of type 'WheelData"
  (cl:format cl:nil "Header header~%std_msgs/Int32 tanSpeed~%std_msgs/Int32 rotSpeed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tanSpeed))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rotSpeed))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelData>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelData
    (cl:cons ':header (header msg))
    (cl:cons ':tanSpeed (tanSpeed msg))
    (cl:cons ':rotSpeed (rotSpeed msg))
))
