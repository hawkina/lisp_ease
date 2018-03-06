(in-package :le)

;; orientation source: cram_knowrob_pick_place

;; grasping offsets -------------------------------------------------------------

(defparameter *lift-z-offset* 0.15 "in meters")

(defparameter *muesli-grasp-z-offset* 0.13 "in meters")
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
;; (defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
;;                                                  object-name
;;                                                  arm
;;                                                  (grasp (eql :top)))
;;   (cl-transforms-stamped:make-transform-stamped
;;    (roslisp-utilities:rosify-underscores-lisp-name object-name)
;;    (ecase arm
;;      (:left cram-tf:*robot-left-tool-frame*)
;;      (:right cram-tf:*robot-right-tool-frame*))
;;    0.0
;;    (cl-transforms:make-3d-vector 0.0d0 0.0d0 *muesli-grasp-z-offset*)
;;    (cl-transforms:matrix->quaternion
;;     #2A((0 1 0)
;;         (1 0 0)
;;         (0 0 -1)))))
;; (defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
;;                                                           object-name
;;                                                           arm
;;                                                           (grasp (eql :top))
;;                                                           grasp-transform)
;;   (cram-tf:translate-transform-stamped grasp-transform :z-offset *lift-z-offset*))

;;BACK grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* ((*transf* nil)
         (*end-transf*))
    (setq *transf*  (cl-tf:transform*
                    (cl-tf:transform-inv
                     (make-poses "?PoseHandStart"))
                     (make-poses "?PoseObjStart") ))
    (setq *end-transf* (cl-transforms-stamped:make-transform-stamped
                        (roslisp-utilities:rosify-underscores-lisp-name object-name)
                        
                        (ecase arm
                          (:left cram-tf:*robot-left-tool-frame*)
                          (:right cram-tf:*robot-right-tool-frame*))
                        0.0
                        ;; (cl-transforms:make-3d-vector *muesli-grasp-xy-offset* 0.0d0 *muesli-grasp-z-offset*)
                        ;; (cl-transforms:matrix->quaternion
                        ;;  #2A((0 0 1)
                        ;;      (1 0 0)
                        ;;      (0 1 0)))
                        ;; (cl-transforms:translation (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))))
                        ;; (cl-transforms:rotation (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))))
                        (cl-transforms:make-3d-vector (+ 0 (cl-tf:x (cl-tf:translation *transf*))) 0.0  (- *muesli-grasp-z-offset* (cl-tf:z (cl-tf:translation *transf*))))
                        (cl-transforms:rotation  (quaternion-w-flip *transf*))))
    (print *end-transf*)))

;;------------
;; TESTER
;; (defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
;;                                                  object-name
;;                                                  arm
;;                                                  (grasp (eql :human-grasp)))
;;   (let* ((*test-transf*))
;;     (print "GRASPING STUFF LIKE A HUMAN.")
;;     (setq *test-transf* (cl-transforms-stamped:make-transform-stamped
;;                          (roslisp-utilities:rosify-underscores-lisp-name object-name)
 
;;                          (ecase arm
;;                            (:left cram-tf:*robot-left-tool-frame*)
;;                            (:right cram-tf:*robot-right-tool-frame*))
;;                          0.0
;;                          (cl-transforms:make-3d-vector *muesli-grasp-xy-offset* 0.0d0 *muesli-grasp-z-offset*)
;;                          (cl-transforms:matrix->quaternion
;;                           #2A((0 0 1)
;;                               (1 0 0)
;;                               (0 1 0)))
;;                          ;; (cl-transforms:translation (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))))
;;                          ;; (cl-transforms:rotation (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))))
;;                          ))
;;     (print *test-transf*)))



(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *muesli-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-muesli))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *muesli-pregrasp-xy-offset*)))

;;; FRONT grasp
;; (defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
;;                                                  object-name
;;                                                  arm
;;                                                  (grasp (eql :front)))
;;   (cl-transforms-stamped:make-transform-stamped
;;    (roslisp-utilities:rosify-underscores-lisp-name object-name)
;;    (ecase arm
;;      (:left cram-tf:*robot-left-tool-frame*)
;;      (:right cram-tf:*robot-right-tool-frame*))
;;    0.0
;;    ;; (cl-transforms:make-3d-vector *muesli-grasp-xy-offset* 0.0d0 *muesli-grasp-z-offset*)
;;    ;; (cl-transforms:matrix->quaternion
;;    ;;  #2A((0 0 -1)
;;    ;;      (-1 0 0)
;;    ;;      (0 1 0)))
;;    (cl-transforms:translation (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))))
;;    (cl-transforms:rotation (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))) )))


;; (defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
;;                                                           object-name
;;                                                           arm
;;                                                           (grasp (eql :front))
;;                                                           grasp-transform)
;;   (cram-tf:translate-transform-stamped grasp-transform :x-offset *muesli-pregrasp-xy-offset*
;;                                                        :z-offset *lift-z-offset*))

;; (defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-muesli))
;;                                                               object-name
;;                                                               arm
;;                                                               (grasp (eql :front))
;;                                                               grasp-transform)
;;   (cram-tf:translate-transform-stamped grasp-transform :x-offset *muesli-pregrasp-xy-offset*))



;;; grasping testing
;; (init-reset-sim)
;; ( move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjStart"))) 'ba-muesli)
;; (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
;; (pick-up-obj)
