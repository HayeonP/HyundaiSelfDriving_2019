
(cl:in-package :asdf)

(defsystem "v2x_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "v2x_info" :depends-on ("_package_v2x_info"))
    (:file "_package_v2x_info" :depends-on ("_package"))
  ))