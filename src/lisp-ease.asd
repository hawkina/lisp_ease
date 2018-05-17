(asdf:defsystem lisp-ease
	:depends-on (roslisp
               cram-language
               cram-json-prolog
               std_msgs-msg
               cl-transforms
               cl-transforms-stamped
               cl-tf
               cram-bullet-world-tutorial)
	:components
	
	((:module "queries"
	  :components
	  ((:file "package")
	   (:file "queries" :depends-on ("package"))
     (:file "basic-queries")
     (:file "plans" :depends-on ("package"))
     (:file "designators" :depends-on ("package"))
     (:file "interfaces" :depends-on ("package"))
     (:file "grasping" :depends-on ("package"))
     (:file "gaussian" :depends-on ("package"))
     (:file "utils" :depends-on ("package"))
     
     (:file "init" :depends-on ("package"))
     (:file "openease-to-bullet" :depends-on ("package"))
     (:file "items" :depends-on ("package"))
     (:file "data-extraction" :depends-on ("package"))
     (:file "map-calibration" :depends-on ("package"))
     
     ))))
