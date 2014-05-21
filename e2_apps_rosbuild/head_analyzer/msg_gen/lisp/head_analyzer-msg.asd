
(cl:in-package :asdf)

(defsystem "head_analyzer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MoveDataMSG" :depends-on ("_package_MoveDataMSG"))
    (:file "_package_MoveDataMSG" :depends-on ("_package"))
    (:file "HeadDataMSG" :depends-on ("_package_HeadDataMSG"))
    (:file "_package_HeadDataMSG" :depends-on ("_package"))
    (:file "EyesDataMSG" :depends-on ("_package_EyesDataMSG"))
    (:file "_package_EyesDataMSG" :depends-on ("_package"))
  ))