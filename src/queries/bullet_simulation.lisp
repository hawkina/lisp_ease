(in-package :le)

(defun init-bullet-world ()
  ;; append own meshes to meshes list so that they can be loaded.
  (append-meshes-to-list)
  
  ;; init tf early. Otherwise there will be excaptions.
  (cram-tf::init-tf)
  
  ;;set costmap parameters
  (prolog:def-fact-group costmap-metadata ()
    (prolog:<- (location-costmap:costmap-size 12 12))
    (prolog:<- (location-costmap:costmap-origin -6 -6))
    (prolog:<- (location-costmap:costmap-resolution 0.05))

    (prolog:<- (location-costmap:costmap-padding 0.2))
    (prolog:<- (location-costmap:costmap-manipulation-padding 0.2))
    (prolog:<- (location-costmap:costmap-in-reach-distance 0.6))
    (prolog:<- (location-costmap:costmap-reach-minimal-distance 0.2)))
  ;; set params
  (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "kitchen_description")

  (sem-map:get-semantic-map)

  (cram-occasions-events:clear-belief)

  (setf cram-tf:*tf-default-timeout* 2.0)

  (setf prolog:*break-on-lisp-errors* t)
  
  ;; initialization from the tutorial
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (assert (btr:object ?world :static-plane :floor ((0 0 0) (0 0 0 1))
                                                  :normal (0 0 1) :constant 0))))
  (cram-occupancy-grid-costmap::init-occupancy-grid-costmap)
  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
 
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
                    (cram-robot-interfaces:robot-arms-carrying-joint-states ?robot ?joint-states)
                    (assert (btr:joint-state ?world ?robot ?joint-states))
                    (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))

 ;spawn kitchen
  (let ((kitchen-urdf 
                 (cl-urdf:parse-urdf 
                  (roslisp:get-param "kitchen_description"))))
             (prolog:prolog
              `(and (btr:bullet-world ?world)
                    (assert (btr:object ?world :semantic-map no-urdf-kitchen ((0 0 0) (0 0 0 1)) ))))))



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
  (btr-utils:move-object 'cereal-4
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

  ;; position all 3 axes,
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
  "simulates the world for a second."
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (btr:simulate ?world 10))))

(defun check-stability-of-sim ()
  "checks if the simulation is stable, or if run for a longer time, some objects would change their position. If the result is anything but NIL, the world is stable."
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (btr:simulate ?world 100))))

(defun poses-demo ()
  (spawn-cereal-at-pose 'cereal-1 (apply-bullet-transform (make-poses "?PoseObjStart")) '(1 1 1))
  
  (spawn-cereal-at-pose 'cereal-2 (apply-bullet-transform (make-poses "?PoseObjStart")) '(1 0 0))
  (spawn-cereal-at-pose 'cereal-3 (apply-bullet-transform (make-poses "?PoseObjEnd")) '(0.6 0 0))

  (spawn-cereal-at-pose 'cereal-4 (apply-bullet-transform (make-poses "?PoseHandStart")) '(0 1 0))
  (spawn-cereal-at-pose 'cereal-5 (apply-bullet-transform (make-poses "?PoseHandEnd")) '(0 0.6 0))

  (spawn-cereal-at-pose 'cereal-6 (apply-bullet-transform (make-poses "?PoseCameraStart")) '(0 0 1))
  (spawn-cereal-at-pose 'cereal-7 (apply-bullet-transform (make-poses "?PoseCameraEnd")) '(0 0 0.6))
  
  ;; move objects like needed
  (move-object (cl-tf:make-transform (cl-tf:make-identity-vector) (cl-tf:make-identity-rotation)) 'cereal-1)
  
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
     (cl-tf:make-quaternion (cl-tf:y quaternion) (cl-tf:z quaternion) (cl-tf:w  quaternion) (cl-tf:x quaternion)))))


;; remove z so we can move the robot to the according position
(defun remove-z (pose)
  (let* ((translation (cl-tf:translation pose)))
    (cl-tf:make-transform
     (cl-tf:make-3d-vector (cl-tf:x translation) (cl-tf:y translation) 0)
     (cl-tf:rotation pose))))

;; closest one so far
(defun apply-bullet-transform (transform)
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

(defun move-all-boxes ()
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjStart"))) 'cereal-2)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseObjEnd"))) 'cereal-3)

  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandStart"))) 'cereal-4)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseHandEnd"))) 'cereal-5)

  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseCameraStart"))) 'cereal-6)
  (move-object (apply-bullet-transform (quaternion-w-flip (make-poses "?PoseCameraEnd"))) 'cereal-7))

;;(make-poses "?PoseCameraStart")
(defun move-robot (transform)
  ;; make the transform a viable robot position
  (let* ((pose (cl-tf:transform->pose  (remove-z (apply-bullet-transform (quaternion-w-flip transform)))))
         (quaternion (cl-tf:orientation pose))
         (x nil)
         (y nil))
    ;; make quaternion
    ;; make into matrix, get x and y values
    (setq x (aref (cl-tf:quaternion->matrix quaternion) 0 2))
    (setq y (aref (cl-tf:quaternion->matrix quaternion) 1 2))
    (setq quaternion (cl-tf:axis-angle->quaternion (cl-tf:make-3d-vector 0 0 1) (atan y x)))
    
    (setq pose (cl-tf:make-pose
                (cl-tf:origin pose)
                quaternion))
    
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world cram-pr2-description:pr2 ,pose))))))

(defun move-head (pose)
  (prolog:prolog `(and (btr:bullet-world ?world)
                       (cram-robot-interfaces:robot ?robot )
                       (btr:head-pointing-at ?world ?robot ,pose))))
;;deprecated
(defun move-head2 (pose)
  (prolog:prolog `(and (btr:bullet-world ?world)
                       (assert (btr:calculate-pan-tilt cram-pr2-description:pr2 ?head_pan_link ?head_tilt_link ,pose)))))

;; this one is the only one that works so far?
;; head keeps moving though?
(defun move-head-test (pose)
(let* ((bullet-pose (remove-z (apply-bullet-transform (quaternion-w-flip pose)))))
  (cram-pr2-projection::look-at-pose-stamped
   (cl-tf:make-pose-stamped
    "map"
    0.0
    (cl-tf:translation bullet-pose)
    (cl-tf:rotation bullet-pose)))))

;;if in back 'cereal-5
(defun is-in-view (name-of-object)
  (prolog:prolog `(and (btr:bullet-world ?world)
                              (cram-robot-interfaces:robot ?robot)
                              (btr:visible ?world ?robot ,name-of-object))))


(defun add-bowl ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ba-bowl ((0 1 2) (0 0 0 1))
                            :mass 0.2 :color (1 1 1) :mesh :ba-bowl)))))
(defun add-cup ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ba-cup ((0 2 2) (0 0 0 1))
                            :mass 0.2 :color (1 1 0) :mesh :ba-cup)))))
(defun add-muesli ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ba-muesli ((0 3 2) (0 0 0 1))
                            :mass 0.2 :color (1 0 1) :mesh :ba-muesli)))))

(defun add-spoon ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ba-spoon ((0 4 2) (0 0 0 1))
                            :mass 0.2 :color (0 0 1) :mesh :ba-spoon)))))
(defun add-milk ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ba-milk ((0 6 2) (0 0 0 1))
                            :mass 0.2 :color (1 0 0) :mesh :ba-milk)))))


;; just when I want to spawn all of them 
(defun spawn-all-own-obj ()
  (add-bowl)
  (add-cup)
  (add-muesli)
  (add-spoon)
  (add-milk))
