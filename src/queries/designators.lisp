(in-package :le)

;;--------------------------- from tutorials ----------------------------

;;(make-poses "?PoseCameraStart")
;; move robot away from the original pose by this offset. 
(defparameter *human-feet-offset* 0.3 )

(defun set-grasp-base-pose (transform)
   ;; make the transform a viable robot position
  (let* ((pose (cl-tf:transform->pose  (remove-z (apply-bullet-transform (quaternion-w-flip transform)))))
         (quaternion (cl-tf:orientation pose))
         (x nil)
         (y nil)
         (?grasp-base-pose nil))
    ;; make quaternion
    ;; make into matrix, get x and y values
    (setq x (aref (cl-tf:quaternion->matrix quaternion) 0 2))
    (setq y (aref (cl-tf:quaternion->matrix quaternion) 1 2))
    (setq quaternion (cl-tf:axis-angle->quaternion (cl-tf:make-3d-vector 0 0 1) (atan y x)))
    
 (setq ?grasp-base-pose 
               (cl-transforms-stamped:make-pose-stamped
                "map"
                0.0
                (cl-tf:make-3d-vector
                 (+ (cl-tf:x (cl-tf:origin pose)) *human-feet-offset*)
                 (cl-tf:y (cl-tf:origin pose))
                 0)
                quaternion))))

;;(make-poses "?PoseObjStart")
(defun set-grasp-look-pose (transform)
  (let* ((?grasp-look-pose nil))
    (setq ?grasp-look-pose
          (cl-transforms-stamped:make-pose-stamped
           "map"
           0.0
           (cl-tf:origin (cl-tf:transform->pose (apply-bullet-transform transform)))
           (cl-tf:orientation (cl-tf:transform->pose (apply-bullet-transform transform)))))))


