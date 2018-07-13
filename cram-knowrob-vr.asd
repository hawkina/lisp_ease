(asdf:defsystem cram-knowrob-vr
	:depends-on (roslisp
               cram-language
               cram-json-prolog
               std_msgs-msg
               cl-transforms
               cl-transforms-stamped
               cl-tf
               cram-bullet-world-tutorial
               cram-knowrob-pick-place)
	:components
	
	((:module "src"
	  :components
	  ((:file "package")
	   (:file "queries" :depends-on ("package"))
     (:file "utility-queries" :depends-on ("package"))           
     (:file "utils" :depends-on ("package"))
     (:file "demo-plans" :depends-on ("package")) ; plans for demonstrations
     
     (:file "init" :depends-on ("package")) ; initialisation
     (:file "openease-to-bullet" :depends-on ("package"))
     (:file "items" :depends-on ("package"))
     (:file "data-extraction" :depends-on ("package"))
     (:file "map-calibration" :depends-on ("package"))
     (:file "plans" :depends-on ("package"))
     (:file "plan-execution" :depends-on ("package"))
     (:file "robot-positions-calculations" :depends-on ("package"))
     (:file "grasping" :depends-on ("package")) ; specifies how to grasp obj
     (:file "gaussian" :depends-on ("package"))
     (:file "demo-preparation" :depends-on ("package"))
     ))))
