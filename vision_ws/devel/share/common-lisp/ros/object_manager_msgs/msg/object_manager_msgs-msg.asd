
(cl:in-package :asdf)

(defsystem "object_manager_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :autoware_msgs-msg
               :jsk_recognition_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "combined" :depends-on ("_package_combined"))
    (:file "_package_combined" :depends-on ("_package"))
  ))