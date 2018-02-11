(in-package :le)

;;------------------- from tutorials ----------------------
;;example call: (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
(defun move-to-object (?grasping-base-pose ?grasping-look-pose)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
  (cpl:top-level 
    (cpl:seq
      (pp-plans::park-arms)
      ;; move the robot to location
      (exe:perform (desig:an action
                            (type going)
                            (target (desig:a location (pose ?grasping-base-pose)))))
      ;; move the head to look at location
      (exe:perform (desig:an action
                            (type looking)
                            (target (desig:a location (pose ?grasping-look-pose)))))))))


(defvar ?desig-saver nil)
;; TODO finish making this universal
(defun pick-up-obj ()
    (let* ((?obj-desig nil)
           (?arm (get-hand)))
        (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (setf ?obj-desig
                   (exe:perform (desig:an action
                                          (type detecting)
                                          (object (desig:an object (type ba-muesli))))))
             (print  (desig:reference
                       (desig:an action
                                 (type picking-up)
                                 (arm ?arm)
                                 (object ?obj-desig))))
             (exe:perform 
              (desig:an action
                        (type picking-up)
                        (arm ?arm)
                        (object ?obj-desig)))
             (setq ?desig-saver ?obj-desig)))))

(defun place-muesli (?arm ?place-pose)
  (let* ()
    (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        (print (desig:a location (pose ?place-pose)))
       (exe:perform
        (desig:an action
                  (type placing)
                  (arm ?arm)
                  ;; (object ?desig-saver)
                  (target (desig:a location (pose ?place-pose)))))))))

;works
(defun move-torso-up (?angle)
   (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (exe:perform
              (desig:a motion (type moving-torso) (joint-angle ?angle))))))



(defun reachability-tester ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level 
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    "base_footprint"
                    0.0
                    (cl-tf:make-3d-vector 0.5102766878660931d0 0.06244204412128673d0 0.8842221563061078d0)
                    (cl-tf:make-quaternion 0.1263345392921927d0 -0.9919876636802037d0 -2.308838915662856d-5 6.0027490527310316d-6))))
        (exe:perform (desig:an motion
                               (type moving-tcp)
                               (left-target (desig:a location (pose ?pose)))))))))

;; from mobile pick place ---------------------------------------------------
(cpl:def-cram-function pick-up (?object-designator
                                ?arm ?gripper-opening  ?grip-effort ?grasp
                                ?left-reach-poses ?right-reach-poses
                                ?left-lift-poses ?right-lift-poses)
  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper")
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)))
    (roslisp:ros-info (pick-place pick-up) "Reaching")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (exe:perform
       (desig:an action
                 (type reaching)
                 (left-poses ?left-reach-poses)
                 (right-poses ?right-reach-poses)))))
  (roslisp:ros-info (pick-place pick-up) "Gripping")
  (exe:perform
   (desig:an action
             (type gripping)
             (gripper ?arm)
             (effort ?grip-effort)
             (object ?object-designator)))
  (roslisp:ros-info (pick-place pick-up) "Assert grasp into knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-attached
     :object-name (desig:desig-prop-value ?object-designator :name)
     :arm ?arm))
  (roslisp:ros-info (pick-place pick-up) "Lifting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type lifting)
               (left-poses ?left-lift-poses)
               (right-poses ?right-lift-poses)))))


; PLACING ----------------------------------------------

(cpl:def-cram-function place (?object-designator
                              ?arm
                              ?left-reach-poses ?right-reach-poses
                              ?left-put-poses ?right-put-poses
                              ?left-retract-poses ?right-retract-poses)
  (roslisp:ros-info (pick-place place) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses))))
  (roslisp:ros-info (pick-place place) "Putting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type putting)
               (left-poses ?left-put-poses)
               (right-poses ?right-put-poses))))
  (roslisp:ros-info (pick-place place) "Opening gripper")
  (exe:perform
   (desig:an action
             (type releasing)
             (object ?object-designator)
             (gripper ?arm)))
  (roslisp:ros-info (pick-place place) "Retract grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached
     :arm ?arm
     :object-name (desig:desig-prop-value ?object-designator :name)))
  (roslisp:ros-info (pick-place place) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type retracting)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses)))))

(cpl:def-cram-function release (?left-or-right)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (pick-and-place release) "~a" e)
         (return)))
    (exe:perform
     (desig:a motion
              (type opening)
              (gripper ?left-or-right)))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))



; DEMO-----------------------------------------------------------------



(defun planning-demo ()
  ;; initialization and preparation
  (init-clean-table)
  (init-bullet-world)
  (add-bowl)
  (add-muesli)
  (get-poses-from-event)

  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjStart"))) 'ba-muesli)

  ; move the robot
  (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
  ; pick up obj - muesli atm
  (pick-up-obj)

  ; move to placing spot
  (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraEnd")) (set-grasp-look-pose (make-poses "?PoseObjEnd")))
  
  ;place
  (place-muesli (get-hand)  (set-grasp-look-pose (make-poses "?PoseObjEnd"))))

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
