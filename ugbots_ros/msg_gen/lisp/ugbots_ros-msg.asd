
(cl:in-package :asdf)

(defsystem "ugbots_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "bin_status" :depends-on ("_package_bin_status"))
    (:file "_package_bin_status" :depends-on ("_package"))
  ))