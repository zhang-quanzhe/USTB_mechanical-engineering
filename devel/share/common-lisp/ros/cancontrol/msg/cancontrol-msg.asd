
(cl:in-package :asdf)

(defsystem "cancontrol-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "vandw" :depends-on ("_package_vandw"))
    (:file "_package_vandw" :depends-on ("_package"))
  ))