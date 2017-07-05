(asdf:defsystem lisp-ease
	:depends-on (roslisp
               cram-language
               cram-json-prolog)
	:components
	((:module "queries"
	  :components
	  ((:file "package")
	   (:file "queries" :depends-on ("package"))))))
