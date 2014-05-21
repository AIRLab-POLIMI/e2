
(cl:in-package :asdf)

(defsystem "user_tracker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Com" :depends-on ("_package_Com"))
    (:file "_package_Com" :depends-on ("_package"))
  ))