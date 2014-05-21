; Auto-generated. Do not edit!


(cl:in-package user_tracker-msg)


;//! \htmlinclude Com.msg.html

(cl:defclass <Com> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (comPoints
    :reader comPoints
    :initarg :comPoints
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (headPoint
    :reader headPoint
    :initarg :headPoint
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (distanceThreshold
    :reader distanceThreshold
    :initarg :distanceThreshold
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32)))
)

(cl:defclass Com (<Com>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Com>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Com)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name user_tracker-msg:<Com> is deprecated: use user_tracker-msg:Com instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Com>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_tracker-msg:header-val is deprecated.  Use user_tracker-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'comPoints-val :lambda-list '(m))
(cl:defmethod comPoints-val ((m <Com>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_tracker-msg:comPoints-val is deprecated.  Use user_tracker-msg:comPoints instead.")
  (comPoints m))

(cl:ensure-generic-function 'headPoint-val :lambda-list '(m))
(cl:defmethod headPoint-val ((m <Com>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_tracker-msg:headPoint-val is deprecated.  Use user_tracker-msg:headPoint instead.")
  (headPoint m))

(cl:ensure-generic-function 'distanceThreshold-val :lambda-list '(m))
(cl:defmethod distanceThreshold-val ((m <Com>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader user_tracker-msg:distanceThreshold-val is deprecated.  Use user_tracker-msg:distanceThreshold instead.")
  (distanceThreshold m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Com>) ostream)
  "Serializes a message object of type '<Com>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'comPoints) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'headPoint) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'distanceThreshold) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Com>) istream)
  "Deserializes a message object of type '<Com>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'comPoints) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'headPoint) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'distanceThreshold) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Com>)))
  "Returns string type for a message object of type '<Com>"
  "user_tracker/Com")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Com)))
  "Returns string type for a message object of type 'Com"
  "user_tracker/Com")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Com>)))
  "Returns md5sum for a message object of type '<Com>"
  "203b33db4f4a438ecfb86d49899a8f2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Com)))
  "Returns md5sum for a message object of type 'Com"
  "203b33db4f4a438ecfb86d49899a8f2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Com>)))
  "Returns full string definition for message of type '<Com>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point32 comPoints~%geometry_msgs/Point32 headPoint~%std_msgs/Int32 distanceThreshold~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Com)))
  "Returns full string definition for message of type 'Com"
  (cl:format cl:nil "Header header~%geometry_msgs/Point32 comPoints~%geometry_msgs/Point32 headPoint~%std_msgs/Int32 distanceThreshold~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Com>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'comPoints))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'headPoint))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'distanceThreshold))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Com>))
  "Converts a ROS message object to a list"
  (cl:list 'Com
    (cl:cons ':header (header msg))
    (cl:cons ':comPoints (comPoints msg))
    (cl:cons ':headPoint (headPoint msg))
    (cl:cons ':distanceThreshold (distanceThreshold msg))
))
