(in-package :le)

;;------------------- from tutorials ----------------------
;;example call: (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
(defun move-to-object (?grasping-base-pose ?grasping-look-pose)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
  (cpl:top-level 
    (cpl:seq
      (pp-plans::park-arms)
      (exe:perform (desig:an action
                            (type going)
                            (target (desig:a location (pose ?grasping-base-pose)))))
      (exe:perform (desig:an action
                            (type looking)
                            (target (desig:a location (pose ?grasping-look-pose)))))))))

(defun pick-up-cereal (&optional (?arm :right))
    (let* ((?cereal-desig nil))
        (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (setf ?cereal-desig
                   (exe:perform (desig:an action
                                          (type detecting)
                                          (object (desig:an object (type ba-muesli))))))
             (print  (desig:reference
                       (desig:an action
                                 (type picking-up)
                                 (arm ?arm)
                                 (object ?cereal-desig))))
             (exe:perform 
              (desig:an action
                        (type picking-up)
                        (arm ?arm)
                        (object ?cereal-desig)))))))             

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


