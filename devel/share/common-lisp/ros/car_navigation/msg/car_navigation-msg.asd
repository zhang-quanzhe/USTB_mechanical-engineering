
(cl:in-package :asdf)

(defsystem "car_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PoseAngle" :depends-on ("_package_PoseAngle"))
    (:file "_package_PoseAngle" :depends-on ("_package"))
  ))