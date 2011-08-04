
(cl:in-package :asdf)

(defsystem "art_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "KinectMsg" :depends-on ("_package_KinectMsg"))
    (:file "_package_KinectMsg" :depends-on ("_package"))
  ))