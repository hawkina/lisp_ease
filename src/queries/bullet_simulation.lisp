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



;spawn cerial box
;one can clean the entire world with this: (roslisp-utilities:startup-ros)
(defun spawn-cereal ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                   ;spawns the cerial at the given pose and quaternion.
                   (assert (btr:object ?world :cereal cereal-3  ((0 0 2) (0 0 0 1))
                                                      :mass 0.2 :color (0 1 1) :size (0.02 0.1 0.1))))))

(defun spawn-cereal-at-pose (name pose)
 (prolog:prolog `(and (btr:bullet-world ?world)
                   ;spawns the cerial at the given pose and quaternion.
                   (assert (btr:object ?world :cereal ,name ,pose
                                                      :mass 0.2 :color (0 0 1) :size (0.02 0.1 0.1)))))
  )

(defun move-cereal (px py pz qx qy qz qw)
  (btr-utils:move-object 'cereal-3
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


(defun move-object (pose obj)
  (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world ,obj ,pose)))))

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
