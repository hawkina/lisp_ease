(in-package :le)

(defun init-bullet-world ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (assert (btr:object ?world :static-plane :floor ((0 0 0) (0 0 0 1))
                                                  :normal (0 0 1) :constant 0))))
  (cram-occupancy-grid-costmap::init-occupancy-grid-costmap)
  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
  (cram-tf::init-tf)
  ;(prolog:prolog '(btr:bullet-world ?world))
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (btr:debug-window ?world)))
  ;load robot description
  (let ((robot-urdf
                   (cl-urdf:parse-urdf
                    (roslisp:get-param "robot_description"))))
             (prolog:prolog
              `(and (btr:bullet-world ?world)
                    (cram-robot-interfaces:robot ?robot)
                    (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot-urdf))
                    (cram-robot-interfaces:robot-arms-parking-joint-states ?robot ?joint-states)
                    (assert (btr:joint-state ?world ?robot ?joint-states))
                    (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))

 ;spawn kitchen
  (let ((kitchen-urdf 
                 (cl-urdf:parse-urdf 
                  (roslisp:get-param "kitchen_description"))))
             (prolog:prolog
              `(and (btr:bullet-world ?world)
                    (assert (btr:object ?world :semantic-map my-no-urdf-kitchen ((0 0 0) (0 0 0 1)) ))))))



;; same as make-pose just for bullet world
(defun make-bullet-pose (pose)
  (list
   (subseq pose 0 3)
   (subseq pose 3 7)))

;; name has to be a string and has to include a ?.
;; ex: ?PoseHandStart
(defun make-bullet-poses (name)
  (make-bullet-pose (cut:var-value (intern name) (get-poses-from-event))))

;spawn cerial box
;one can clean the entire world with this: (roslisp-utilities:startup-ros)
(defun spawn-cereal ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                   ;spawns the cerial at the given pose and quaternion.
                   (assert (btr:object ?world :cereal cereal-3  ((0 0 2) (0 0 0 1))
                                                      :mass 0.2 :color (0 1 1) :size (0.02 0.1 0.1))))))

(defun spawn-cereal-at-pose (name transform color)
  (let* ((pose (cl-tf:transform->pose transform))) 
    (prolog:prolog `(and (btr:bullet-world ?world)
                                        ;spawns the cerial at the given pose and quaternion.
                         (assert (btr:object ?world :cereal ,name ,pose
                                             :mass 0.2 :color ,color :size (0.02 0.1 0.1)))))))

(defun move-cereal (px py pz qx qy qz qw)
  (btr-utils:move-object 'cereal-2
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector px py pz)
                          (cl-transforms:make-quaternion qx qy qz qw))))

; spawns x y z axes at the given point. the factor is the offset needed, since otherwise the point would be within the spawned axis object
(defun spawn-axes (x y z factor)
  ;;spawn all 3 axes
  ;; x-Axis
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :cereal x-axis  ((0 0 2) (0 0 0 1))
                                                      :mass 0.0 :color (1 0 0) :size (0.2 0.02 0.02)))))
  ;; y-Axis
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :cereal y-axis  ((0 0 2) (0 0 0 1))
                                                      :mass 0.0 :color (0 1 0) :size (0.02 0.2 0.02)))))
  ;; z-Axis
   (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :cereal z-axis  ((0 0 2) (0 0 0 1))
                                                      :mass 0.0 :color (0 0 1) :size (0.02 0.02 0.2)))))

  ;; position all 3 axes
  (btr-utils:move-object 'x-axis
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector (+ x factor) y z)
                          (cl-transforms:make-identity-rotation)))
  (btr-utils:move-object 'y-axis
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector x (+ y factor) z)
                          (cl-transforms:make-identity-rotation)))
  (btr-utils:move-object 'z-axis
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector x y (+ z factor))
                          (cl-transforms:make-identity-rotation)))
  
  )

(defun place-rotate-axes (px py pz pfactor qx qy qz qw)
  (btr-utils:move-object 'x-axis
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector (+ px pfactor) py pz)
                          (cl-transforms:make-quaternion qx qy qz qw)))
  (btr-utils:move-object 'y-axis
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector px (+ py pfactor) pz)
                          (cl-transforms:make-quaternion qx qy qz qw)))
  (btr-utils:move-object 'z-axis
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector px py (+ pz pfactor))
                          (cl-transforms:make-quaternion qx qy qz qw))))


(defun move-object (transform obj)
  (let* ((pose (cl-tf:transform->pose transform)))
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world ,obj ,pose))))))

(defun test-move-object ()
  (let ((pose))
    (progn
      (setq pose (test-pose-lists-parser))
      (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world mug-2 ,pose)))))))

; usecase: (move-object (pose-lists-parser '|?PoseObjEnd|))
; moves object to the pose.


(defun check-obj-in-world (object-name)
  (btr:object btr:*current-bullet-world* object-name))

(defun start-sim ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (btr:simulate ?world 10))))

(defun poses-demo ()
  (spawn-cereal-at-pose 'cereal-1 (apply-bullet-transform (make-poses "?PoseObjStart")) '(1 1 1))
  
  (spawn-cereal-at-pose 'cereal-2 (apply-bullet-transform (make-poses "?PoseObjStart")) '(1 0 0))
  (spawn-cereal-at-pose 'cereal-3 (apply-bullet-transform (make-poses "?PoseObjEnd")) '(0.6 0 0))

  (spawn-cereal-at-pose 'cereal-4 (apply-bullet-transform (make-poses "?PoseHandStart")) '(0 1 0))
  (spawn-cereal-at-pose 'cereal-5 (apply-bullet-transform (make-poses "?PoseHandEnd")) '(0 0.6 0))

  (spawn-cereal-at-pose 'cereal-6 (apply-bullet-transform (make-poses "?PoseCameraStart")) '(0 0 1))
  (spawn-cereal-at-pose 'cereal-7 (apply-bullet-transform (make-poses "?PoseCameraEnd")) '(0 0 0.6))
  
  ;; move objects like needed
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjStart"))) 'cereal-2)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjEnd"))) 'cereal-3)

  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))) 'cereal-4)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandEnd"))) 'cereal-5)

  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseCameraStart"))) 'cereal-6)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseCameraEnd"))) 'cereal-7)
  )


;; converts the y-value of the "3d-vector" to it's - value
;; Careful! If given a pose which is stored in a variable,
;; the variable will be overwritten
(defun swap-bullet-y-axis (pose)
  (let* ((vector (first pose)))
    (setf (second vector)
          (- (second vector)))
    (list vector
          (second pose))))

;; for proper transforms
(defun swap-y-axis (pose)
  (let* ((vector (cl-tf:translation pose)))
    (cl-tf:make-transform
     (cl-tf:make-3d-vector (cl-tf:x vector) (- (cl-tf:y vector)) (cl-tf:z vector))
     (cl-tf:rotation pose))))

(defun swap-x-axis (pose)
  (let* ((vector (cl-tf:translation pose)))
    (cl-tf:make-transform
     (cl-tf:make-3d-vector (-  (cl-tf:x vector)) (cl-tf:y vector) (cl-tf:z vector))
     (cl-tf:rotation pose))))

(defun quaternion-w-flip (pose)
  (let* ((quaternion (cl-tf:rotation pose)))
    (cl-tf:make-transform
     (cl-tf:translation pose)
     (cl-tf:make-quaternion (cl-tf:y quaternion) (cl-tf:z quaternion) (cl-tf:w  quaternion) (cl-tf:x quaternion))))

  )


;; closest one so far
(defun apply-bullet-transform (transform)
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector -2.7 -1.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) ;; about 230°?
   transform))

(defun move-all-boxes ()
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjStart"))) 'cereal-2)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjEnd"))) 'cereal-3)

  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))) 'cereal-4)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandEnd"))) 'cereal-5)

  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseCameraStart"))) 'cereal-6)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseCameraEnd"))) 'cereal-7))
