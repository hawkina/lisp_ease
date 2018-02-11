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


;; --- DESIG FACT GROUP --------------------------------------------------------------------

(cram-prolog:def-fact-group ba-pnp-object-knowledge (object-rotationally-symmetric orientation-matters object-type-grasp)

  (cram-prolog:<- (object-rotationally-symmetric ?object-type)
    (member ?object-type (:ba-muesli :ba-milk :ba-spoon :ba-cup  :ba-bowl)))

  (cram-prolog:<- (orientation-matters ?object-type)
    (member ?object-type (:ba-spoon)))

  (cram-prolog:<- (object-type-grasp :ba-spoon :top))
  ;; (<- (object-type-grasp :spoon :top))
  ;; (<- (object-type-grasp :fork :top))
  ;; (<- (object-type-grasp :knife :top))

  (cram-prolog:<- (object-type-grasp :ba-milk :side))
  (cram-prolog:<- (object-type-grasp :ba-milk :front))

  (cram-prolog:<- (object-type-grasp :ba-cup :front))
  ;; (<- (object-type-grasp :cup :side))
  ;; (<- (object-type-grasp :cup :top))

  ;; (<- (object-type-grasp :cereal :top))
  (cram-prolog:<- (object-type-grasp :ba-muesli :back))
  (cram-prolog:<- (object-type-grasp :ba-muesli :front))
  ;; (<- (object-type-grasp :breakfast-cereal :top))
  ;; (<- (object-type-grasp :breakfast-cereal :back))
  ;; (<- (object-type-grasp :breakfast-cereal :front))

  (cram-prolog:<- (object-type-grasp :ba-bowl :top)))
