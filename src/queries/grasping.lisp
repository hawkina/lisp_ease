(in-package :le)

;; orientation source: cram_knowrob_pick_place

;; grasping offsets -------------------------------------------------------------

(defparameter *lift-z-offset* 0.15 "in meters")

(defparameter *muesli-grasp-z-offset* 0.04 "in meters")
(defparameter *muesli-grasp-xy-offset* -0.03 "in meters")
(defparameter *muesli-pregrasp-xy-offset* 0.15 "in meters")

;; grasping force in Nm --------------------------------------------------------
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-muesli))) 15)

;; gripper oppening ------------------------------------------------------------
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-muesli))) 0.1)

;;-------------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-lift-transform (object-type object-name
                                                      arm grasp grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :z-offset *lift-z-offset*))

;; grasps -----------------------------------------------------------------------

;; --- MUESLI ---

;;; TOP grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
                                                 object-name
                                                 arm
                                                 (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 *muesli-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (1 0 0)
        (0 0 -1)))))
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
                                                          object-name
                                                          arm
                                                          (grasp (eql :top))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :z-offset *lift-z-offset*))

;;; BACK grasp
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
   (cl-transforms:make-3d-vector *muesli-grasp-xy-offset* 0.0d0 *muesli-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 0 1)
        (1 0 0)
        (0 1 0)))))
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
                                                          object-name
                                                          arm
                                                          (grasp (eql :back))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *muesli-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-muesli))
                                                              object-name
                                                              arm
                                                              (grasp (eql :back))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *muesli-pregrasp-xy-offset*)))

;;; FRONT grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
                                                 object-name
                                                 arm
                                                 (grasp (eql :front)))
  (cl-transforms-stamped:make-transform-stamped
   (roslisp-utilities:rosify-underscores-lisp-name object-name)
   (ecase arm
     (:left cram-tf:*robot-left-tool-frame*)
     (:right cram-tf:*robot-right-tool-frame*))
   0.0
   (cl-transforms:make-3d-vector *muesli-grasp-xy-offset* 0.0d0 *muesli-grasp-z-offset*)
   (cl-transforms:matrix->quaternion
    #2A((0 0 -1)
        (-1 0 0)
        (0 1 0)))))
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
                                                          object-name
                                                          arm
                                                          (grasp (eql :front))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset *muesli-pregrasp-xy-offset*
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-muesli))
                                                              object-name
                                                              arm
                                                              (grasp (eql :front))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset *muesli-pregrasp-xy-offset*))



