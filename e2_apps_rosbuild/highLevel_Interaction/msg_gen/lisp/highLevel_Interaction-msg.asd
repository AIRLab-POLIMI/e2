
(cl:in-package :asdf)

(defsystem "highLevel_Interaction-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SpeakData" :depends-on ("_package_SpeakData"))
    (:file "_package_SpeakData" :depends-on ("_package"))
  ))