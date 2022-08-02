
(cl:in-package :asdf)

(defsystem "just_drone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "dimensions" :depends-on ("_package_dimensions"))
    (:file "_package_dimensions" :depends-on ("_package"))
    (:file "dimensions" :depends-on ("_package_dimensions"))
    (:file "_package_dimensions" :depends-on ("_package"))
  ))