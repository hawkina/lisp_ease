(in-package :le)

;; orientation source: cram_knowrob_pick_place

;; grasping offsets -------------------------------------------------------------

(defparameter *transf-l-wrist-l-fingertip* (cl-tf:make-transform
                                           (cl-tf:make-3d-vector 0.16827996833489822d0 0.014949973681403161d0 -2.010673427754739d-8)
                                           (cl-tf:make-quaternion 5.551115123125783d-17 5.551115123125783d-17 0.0d0 1.0000000000000002d0)))

(defparameter *lift-z-offset* 0.15 "in meters")

(defparameter *muesli-grasp-z-offset* 0.0 "in meters") ; 0.13
(defparameter *muesli-grasp-xy-offset* 0.0 "in meters") ; -0.03
(defparameter *muesli-pregrasp-xy-offset* 0.0 "in meters")  ; 0.15

(defparameter *milk-grasp-xy-offset* -0.20 "in meters")
(defparameter *milk-grasp-z-offset* 0.0 "in meters")
(defparameter *milk-pregrasp-xy-offset* 0.15 "in meters")

(defparameter *bowl-grasp-xy-offset* 0.1 "in meters")
(defparameter *bowl-grasp-z-offset* 0.0 "in meters")
(defparameter *bowl-pregrasp-xy-offset* 0.15 "in meters")

(defparameter *cup-grasp-z-offset* -0.0 "in meters") ; 0.13
(defparameter *cup-grasp-xy-offset* -0.0 "in meters") ; -0.03
(defparameter *cup-pregrasp-xy-offset* 0.0 "in meters")

;; grasping force in Nm --------------------------------------------------------
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-muesli))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-milk))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-bowl))) 15)
(defmethod get-object-type-gripping-effort ((object-type (eql :ba-cup))) 15)

;; gripper oppening ------------------------------------------------------------
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-muesli))) 0.2)
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-milk))) 0.2)
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-bowl))) 0.2)
(defmethod get-object-type-gripper-opening ((object-type (eql :ba-cup))) 0.2)
(defmethod get-object-type-gripper-opening (object-type)
  "Default value is 0.10. In meters."
  0.10)
;;-------------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-lift-transform (object-type object-name
                                                      arm grasp grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :z-offset *lift-z-offset*))

;; grasps -----------------------------------------------------------------------

;; --- MUESLI ---

;;HUMAN grasp
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* ((*transf* nil)
         (*end-transf*))
    (setq *transf*  
          (cl-tf:transform*
           (cl-tf:transform-inv
            (make-poses "?PoseHandStart"))
           (make-poses "?PoseObjStart") ))
    (setq *end-transf* (cl-transforms-stamped:make-transform-stamped
                        (roslisp-utilities:rosify-underscores-lisp-name object-name)
                        
                        (ecase arm
                          (:left cram-tf:*robot-left-tool-frame*)
                          (:right cram-tf:*robot-right-tool-frame*))
                        0.0
                        (cl-transforms:make-3d-vector  (cl-tf:x (cl-tf:translation *transf*)) 0.0  (- *muesli-grasp-z-offset* (cl-tf:z (cl-tf:translation *transf*))))
                        (cl-transforms:make-quaternion  (cl-tf:x (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:y (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:z (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:w (cl-tf:rotation (quaternion-w-flip *transf*))))
                        ;; (cl-transforms:make-quaternion 0.5 0.5 0.5 0.5 )
                        ))
    (print "THIS IS THE END TRANSFORM:")
    (print *end-transf*)))

;;HUMAN OTHER SIDE grasp 
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-muesli))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-other-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* ((*transf* nil)
         (*end-transf*))
    ;; transf. from Map to Obj?
    (setq *transf*  (cl-tf:transform*
                     (cl-tf:transform*
                      (cl-tf:transform-inv
                       (make-poses "?PoseHandStart"))
                      (make-poses "?PoseObjStart"))
                     *transf-l-wrist-l-fingertip*))
                     
    (setq *end-transf* (cl-transforms-stamped:make-transform-stamped
                        (roslisp-utilities:rosify-underscores-lisp-name object-name)
                        
                        (ecase arm
                          (:left cram-tf:*robot-left-tool-frame*)
                          (:right cram-tf:*robot-right-tool-frame*))
                        0.0
                        (cl-transforms:make-3d-vector
                         (+ 0.00 (cl-tf:x (cl-tf:translation *transf*)))
                         0.0
                         (- *muesli-grasp-z-offset* (cl-tf:z (cl-tf:translation *transf*))))

                        (cl-tf:rotation (quaternion-w-flip *transf*))
                        ;; (cl-transforms:make-quaternion 0.5 0.5 0.5 0.5 )
                        ))
    (print "------------------------------")
    (print "THIS IS THE END TRANSFORM:")
    (print "------------------------------")
    (print *end-transf*)))

;;----------------------------------------------------------------------------------------------------
;;  MUESLI
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

;---

(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-muesli))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-other-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *muesli-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))


(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-muesli))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-other-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *muesli-pregrasp-xy-offset*)))

;; MILK
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-milk))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *milk-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-milk))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *milk-pregrasp-xy-offset*)))






;; MILK ----------------------------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-milk))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* ((*transf* nil)
         (*end-transf*))
    (setq *transf*  (apply-rotation-x (cl-tf:transform*
                                       (cl-tf:transform-inv
                                        (make-poses "?PoseHandStart"))
                                       (make-poses "?PoseObjStart") )))
    (setq *end-transf* (cl-transforms-stamped:make-transform-stamped
                        (roslisp-utilities:rosify-underscores-lisp-name object-name)
                        
                        (ecase arm
                          (:left cram-tf:*robot-left-tool-frame*)
                          (:right cram-tf:*robot-right-tool-frame*))
                        0.0
                        (cl-transforms:make-3d-vector  (cl-tf:x (cl-tf:translation *transf*)) 0.0  (- *milk-grasp-z-offset* (cl-tf:z (cl-tf:translation *transf*))))
                        (cl-transforms:make-quaternion  (cl-tf:x (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:y (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:z (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:w (cl-tf:rotation (quaternion-w-flip *transf*))))
                        ;; (cl-transforms:make-quaternion 0.5 0.5 0.5 0.5 )
                        ))
    (print *end-transf*)))

; BOWL -------------------------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-bowl))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* ((*transf* nil)
         (*end-transf*))
    (setq *transf*  (apply-rotation (apply-rotation-x (cl-tf:transform*
                                         (cl-tf:transform-inv
                                          (make-poses "?PoseHandStart"))
                                         (make-poses "?PoseObjStart") ))))
    (setq *end-transf* (cl-transforms-stamped:make-transform-stamped
                        (roslisp-utilities:rosify-underscores-lisp-name object-name)
                        
                        (ecase arm
                          (:left cram-tf:*robot-left-tool-frame*)
                          (:right cram-tf:*robot-right-tool-frame*))
                        0.0
                        (cl-transforms:make-3d-vector  (cl-tf:x (cl-tf:translation *transf*)) 0.0  (- *bowl-grasp-z-offset* (cl-tf:z (cl-tf:translation *transf*))))
                        (cl-transforms:make-quaternion  (cl-tf:x (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:y (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:z (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:w (cl-tf:rotation (quaternion-w-flip *transf*))))
                        ;; (cl-transforms:make-quaternion 0.5 0.5 0.5 0.5 )
                        ))
    (print *end-transf*)))

;;---
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-bowl))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *bowl-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-bowl))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *bowl-pregrasp-xy-offset*)))




;------------------------------------------------------------------------------------------------
; CUP --------------------------------------------------------------------------------------------
(defmethod get-object-type-to-gripper-transform ((object-type (eql :ba-cup))
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))
  (print "GRASPING STUFF LIKE A HUMAN.")
  (let* ((*transf* nil)
         (*end-transf*))
    (setq *transf*  (cl-tf:transform*
                        (cl-tf:transform-inv
                         (make-poses "?PoseObjStart"))
                        (make-poses "?PoseHandStart") ))
    (setq *end-transf* (cl-transforms-stamped:make-transform-stamped
                        (roslisp-utilities:rosify-underscores-lisp-name object-name)
                        
                        (ecase arm
                          (:left cram-tf:*robot-left-tool-frame*)
                          (:right cram-tf:*robot-right-tool-frame*))
                        0.0
                        (cl-transforms:make-3d-vector  (cl-tf:x (cl-tf:translation *transf*)) 0.0  (- *cup-grasp-z-offset* (cl-tf:z (cl-tf:translation *transf*))))
                        (cl-transforms:make-quaternion  (cl-tf:x (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:y (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:z (cl-tf:rotation (quaternion-w-flip *transf*)))
                                                        (cl-tf:w (cl-tf:rotation (quaternion-w-flip *transf*))))
                        ;; (cl-transforms:make-quaternion 0.5 0.5 0.5 0.5 )
                        ))
    (print *end-transf*)))

;;---
(defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql :ba-cup))
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *cup-pregrasp-xy-offset*)
                                                       :z-offset *lift-z-offset*))
(defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql :ba-cup))
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped grasp-transform :x-offset (- *cup-pregrasp-xy-offset*)))




;----------------------------------------------------------------------------------------------------




;;; grasping testing
;; (init-reset-sim)
;; ( move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjStart"))) 'ba-muesli)
;; (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
;; (pick-up-obj)

(defun test ()
  (init-reset-sim)
(move-object (apply-bullet-transform-start (quaternion-w-flip (make-poses "?PoseObjStart"))) 'ba-muesli)
(move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
;;; (move-object (cl-tf:make-transform (cl-tf:make-3d-vector -1.099926662445068d0 -1.210024490952492d0 0.884222149848938d0 ) (cl-tf:make-quaternion -0.5 0 0 0.5)) 'ba-milk)
)


;;---------usefull stuff-------------
;; HOW TO LOOK UP TF TRANSFORM
;; (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
;;        (cram-tf::lookup-transform cram-tf::*transformer* "l_wrist_roll_link" "l_gripper_l_finger_tip_link" ))

(defparameter *transf-map-to-l-wrist* (cl-tf:make-transform (cl-tf:make-3d-vector 0.3980244408936592d0 1.3354333200064086d0 1.0504352569580078d0)
                                                        (cl-tf:make-quaternion 0.023091277107596397d0 -0.04308567941188812d0 0.06526759266853334d0 0.9966697096824646d0)))

(defparameter *transf-map-to-l-finger* (cl-tf:make-transform (cl-tf:make-3d-vector 0.5622712211176504d0 1.3717984540877128d0 1.06599915822347d0)
                                                        (cl-tf:make-quaternion 0.023091277107596397d0 -0.04308567941188812d0 0.06526759266853334d0 0.9966697096824646d0)))
