
(cl:in-package :asdf)

(defsystem "autonomous_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Vector3Array" :depends-on ("_package_Vector3Array"))
    (:file "_package_Vector3Array" :depends-on ("_package"))
  ))