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

(defun pick-up-cereal (&optional (?arm :left))
    (let* ((?cereal-desig nil))
        (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (setf ?cereal-desig
             (exe:perform (desig:an action
                                    (type detecting)
                                    (object (desig:an object (type cereal))))))
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






;---------------------------------------------------------------

(defvar *perceived-obj* nil)
(defun get-perceived-cereal-desig ()
  (let* ((?cereal-desig nil))
        (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (setf ?cereal-desig
                   (pp-plans::perceive (desig:an object (type cereal))))))))



;works
(defun move-torso-up ()
   (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (exe:perform
              (desig:a motion (type moving-torso) (joint-angle 0.3))))))





;;; doesn't work
(defun pick-up (?object-designator &optional (?arm :left))
  (exe:perform (desig:an action
                         (type picking-up)
                         (arm ?arm)
                         (object ?object-designator))))

(defun test ()
(proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (let ((?perceived-obj (get-perceived-cereal-desig)))
        (pick-up ?perceived-obj :left)
  ))))










;;---------------------------------------------------------------
(defun get-transform-base-footprint-to-map ()
  (cram-projection::projection-environment-result-result (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment (cl-tf:lookup-transform cram-tf:*transformer* "map" "base_footprint"))))

(defun get-transform-perceived-obj ()
  (cpl:top-level
    (cram-projection::projection-environment-result-result
     (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
       (pp-plans::perceive (desig:an object (type cereal)))))))

(defun footprint-to-map-transform ()
  (cl-transforms-stamped:transform*
   (get-transform-base-footprint-to-map)
   (get-transform-perceived-obj)))

;how to not hard code this
(defun perceive-obj ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (pp-plans::perceive (desig:an object (type cereal))))))



(defun grasp-obj ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
  (cpl:top-level
    (let ((?perceived-obj-desig nil))
      ;(exe:perform
      ;(desig:a motion (type moving-torso) (joint-angle 0.3)))
      (setq *perceived-obj* (perceive-obj))
   ;   (setq ?perceived-obj-desig ?test)
          
      (exe:perform
       (desig:an action
                 (type picking-up)
                 (object ?perceived-obj-desig)
                 (arm right)))))))

