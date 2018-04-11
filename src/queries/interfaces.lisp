(in-package :le)

;; parameters for grasping cereal from cram_knowrob/cram_knowrob_pick_place/grasp.lisp
(defparameter *lift-z-offset* 0.15 "in meters")

(defparameter *cereal-grasp-z-offset* 0.04 "in meters")
(defparameter *cereal-grasp-xy-offset* -0.03 "in meters")
(defparameter *cereal-pregrasp-xy-offset* 0.15 "in meters")

;;defining own effort depending on obj. orientation cram_knowrob/cram_knowrob_pick_place/grasp.lisp
(defmethod get-object-type-gripping-effort ((object-type (eql :cereal))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :edeka-red-bowl))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :cup-eco-orange))) 50)
(defmethod get-object-type-gripping-effort ((object-type (eql :koelln-muesli-knusper-honig-nuss))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :spoon-blue-plastic))) 100)
(defmethod get-object-type-gripping-effort ((object-type (eql :weide-milch-small))) 15)


;;; FRONT grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cereal))
                                                 object-name
                                                 arm
                                                 (grasp (eql :front)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector *cereal-grasp-xy-offset* 0.0d0 *cereal-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 0 -1)
        (-1 0 0)
        (0 1 0)))))

(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :cereal))
                                                          object-name
                                                          arm
                                                          (grasp (eql :front))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset *cereal-pregrasp-xy-offset*
                                                       :z-offset *lift-z-offset*))

(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :cereal))
                                                              object-name
                                                              arm
                                                              (grasp (eql :front))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset *cereal-pregrasp-xy-offset*))

;; back grasp

(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
                                                 object-name
                                                 arm
                                                 (grasp (eql :back)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector *cereal-grasp-xy-offset* 0.0d0 *cereal-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 0 1)
        (1 0 0)
        (0 1 0)))))
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
                                                          object-name
                                                          arm
                                                          (grasp (eql :back))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *cereal-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-muesli))
                                                              object-name
                                                              arm
                                                              (grasp (eql :back))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *cereal-pregrasp-xy-offset*)))

;; top grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :cereal))
                                                 object-name
                                                 arm
                                                 (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 *cereal-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (1 0 0)
        (0 0 -1)))))
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :cereal))
                                                          object-name
                                                          arm
                                                          (grasp (eql :top))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :z-offset *lift-z-offset*))
