
(cl:in-package :asdf)

(defsystem "robot_brain-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HighLevelData" :depends-on ("_package_HighLevelData"))
    (:file "_package_HighLevelData" :depends-on ("_package"))
    (:file "WheelData" :depends-on ("_package_WheelData"))
    (:file "_package_WheelData" :depends-on ("_package"))
  ))