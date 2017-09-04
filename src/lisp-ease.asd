(asdf:defsystem lisp-ease
	:depends-on (roslisp
               cram-language
               cram-json-prolog
               std_msgs-msg
               cl-tf)
	:components
	((:module "queries"
	  :components
	  ((:file "package")
	   (:file "queries" :depends-on ("package"))
     (:file "basic-queries")
     (:file "combined-queries")))))
