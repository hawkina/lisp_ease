(in-package :le)
;; usefull bullet world infos

;; get-info of an object
(defun get-info (infoObj)
  (cut:var-value (intern infoObj)  poses-list))

;; returns the hand used in the curretnly loaded episode
(defun get-hand ()
  (if (search "Left" (string (get-info "?HandInstShortName")))
      :left
      (if (search "Right" (string (get-info "?HandInstShortName")))
          :right
          NIL)))


;; killing object from bullet world. ex: 'ba-axes
(defun kill-obj (object) 
  (btr-utils:kill-object object)) 

;;-------------------------------------------------------------------------------------
;; all kind of transform utils

(defun make-transform-hand-std-pr2 ()
  (cl-tf:transform* (make-poses "?PoseHandStart")
                    (human-to-right-robot-hand-transform)
                    cram-pr2-description::*standard-to-pr2-gripper-transform*))

(defun get-robot-in-map-pose () 
  (cl-tf:transform->transform-stamped "map" "base_footprint" 0.0
                                      (cl-tf:pose->transform
                                       (btr:pose
                                        (btr:get-robot-object)))))

;; the given pose is the one copy pasted from where he tries to go. this is for vvisualization
(defun make-map-to-gripper-transform ()
  (cl-tf:transform*
   (get-robot-in-map-pose)
   (cl-tf:make-stamped-transform "base_footprint" "r_gripper_tool_frame" 0.0 
                                 (cl-tf:make-3d-vector 0.6207556803418275d0 -0.09483694591892577d0 0.9474515448891849d0)
                                 (cl-tf:make-quaternion -0.08704018159324084d0 -0.0678501392353959d0 -0.912502320545953d0 -0.3939033077712149d0)))
  )

(defun make-transf-manually ()
  ;; inverse of map to object, therefore object to map
  ;; object T map
  (cram-tf:transform-stamped-inv  (cl-tf:transform->stamped-transform "map" "ba_muesli"  0.0 (make-poses "?PoseObjStart")))

  ;; map T unreal hand
  (cl-tf:transform->transform-stamped "map" "hand" 0.0 (make-poses "?PoseHandStart"))

  ;; unreal hand T standard gripper
  (human-to-right-robot-hand-transform)
  )






;; look up transfrom from tf. ex: "l_wrist_roll_link" "l_gripper_l_finger_tip_link" 
(defun lookup-tf-transform (parent_frame child_frame)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
    (cram-tf::lookup-transform cram-tf::*transformer* parent_frame child_frame)))
;;------------------------------------------------------------------------------------------
;; plans

(defun alternative-demo (object)
  (move-object  (make-poses "?PoseObjStart") object)
  (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
  )




;; placing doesnt work in the conventional way. but calling this in the repl works:
;; (defun placing-b ()
;;   (defvar ?test (set-place-pose (make-poses "?PoseObjEnd")))
;;    (let* ()
;;     (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
;;       (cpl:top-level
;;        (exe:perform
;;         (desig:an action
;;                   (type placing)
;;                   (arm :left)
;;                   (object ?desig-saver)
;;                   (target (desig:a location (pose ?test)))))))))

;;-------------------------------------------------------------------------------------------
;; old bullet transform adapting and rotating stuff
;; closest one so far
(defun apply-bullet-transform-start (transform)
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector -2.6 -1.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) 
   transform))

(defun apply-bullet-transform-end (transform)
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector -3.0 -0.5 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) 
   transform))

(defun apply-bullet-transform-old (transform)
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector -2.7 -1.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) 
   transform))

(defun apply-bullet-rotation (transform)
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) 
   transform))

(defun apply-rotation (transform)
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          (/ pi 2))) 
   transform))


;; flip y axes and y of quaternion
(defun flip-left-to-right-handedness (transform)
  (cl-tf:make-transform
   (cl-tf:translation transform)
   (cl-tf:make-quaternion
    (cl-tf:x (cl-tf:rotation transform))
    (- (cl-tf:y (cl-tf:rotation transform)))
    (cl-tf:z (cl-tf:rotation transform))
    (cl-tf:w (cl-tf:rotation transform)))))

;;- more bullet checking/utils stuff
(defun check-obj-in-world (object-name)
  (btr:object btr:*current-bullet-world* object-name))


(defun start-sim ()
  "simulates the world for a second."
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:simulate ?world 10))))

(defun check-stability-of-sim ()
  "checks if the simulation is stable, or if run for a longer time, some objects would change their position. If the result is anything but NIL, the world is stable."
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:simulate ?world 100))))

;; just when I want to spawn all of them 
(defun spawn-all-own-obj ()
  (add-bowl)
  (add-cup)
  (add-muesli)
  (add-spoon)
  (add-milk))

(defun set-axes ()
  (let* ((transf_r)
         (transf_l))
    (setq transf_r (car
                    (cram-projection::projection-environment-result-result
                     (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
                       (cram-tf::lookup-transform cram-tf::*transformer* "map" "r_gripper_r_finger_tip_link" )))))
    (setq transf_l (car
                    (cram-projection::projection-environment-result-result
                     (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
                       (cram-tf::lookup-transform cram-tf::*transformer* "map" "l_gripper_l_finger_tip_link" )))))
    
    (setq transf_r
          (cl-tf:make-transform
           (cl-tf:translation transf_r)
           (cl-tf:rotation transf_r)))
    (move-object transf_r 'ba-axes)
    (setq transf_l
          (cl-tf:make-transform
           (cl-tf:translation transf_l)
           (cl-tf:rotation transf_l)))
    
    (move-object transf_r 'ba-axes)
    (move-object transf_l 'ba-axes2)
    (move-object (make-poses "?PoseHandStart") 'ba-axes3)))

                                        ;splits the list of the pose into pose and quaternion
                                        ;for specific usecase test function
(defun test-pose-lists-parser ()
  (let ((temp))
    (progn
      (setq temp (cut:var-value '|?PoseCameraStart| poses-list))
      (list (subseq temp 0 3)
            '(0 0 1 0)))))



(defun pose-lists-parser (obj)
  (let ((temp))
    (progn
      (setq temp (cut:var-value obj poses-list))
      (list (subseq temp 0 3)
            (subseq temp 3 7)))))
