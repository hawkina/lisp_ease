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
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "no_kitchen_description")

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

 ;; ;spawn kitchen
 ;;  (let ((kitchen-urdf 
 ;;                 (cl-urdf:parse-urdf 
 ;;                  (roslisp:get-param "kitchen_description"))))
 ;;             (prolog:prolog
 ;;              `(and (btr:bullet-world ?world)
 ;;                    (assert (btr:object ?world :semantic-map no-urdf-kitchen ((0 0 0) (0 0 0 1)) )))))
 )



;; same as make-pose just for bullet world
(defun make-bullet-pose (pose)
  (list
   (subseq pose 0 3)
   (subseq pose 3 7)))

;; name has to be a string and has to include a ?.
;; ex: ?PoseHandStart
(defun make-bullet-poses (name)
  (make-bullet-pose (cut:var-value (intern name) (get-poses-from-event))))

(defun move-object (transform obj)
  (let* ((pose (cl-tf:transform->pose transform)))
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world ,obj ,pose))))))

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


;; flip y axes and y of quaternion
(defun flip-left-to-right-handedness (transform)
  (cl-tf:make-transform
   (cl-tf:translation transform)
   (cl-tf:make-quaternion
    (cl-tf:x (cl-tf:rotation transform))
    (- (cl-tf:y (cl-tf:rotation transform)))
    (cl-tf:z (cl-tf:rotation transform))
    (cl-tf:w (cl-tf:rotation transform)))))

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

(defun apply-rotation-x (transform)
    (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 1 0 0)
                          (/ pi 2))) 
   transform))

(defun apply-rotation-tx (transform)
    (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 1 0 0)
                          -1.5)) 
   transform))

(defun apply-rotation-ty (transform)
    (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 1 0)
                          -0.5))
    transform))

(defun apply-rotation-tz (transform)
    (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          1.0))
    transform))


;;(make-poses "?PoseCameraStart")
(defun move-robot (transform)
  ;; make the transform a viable robot position
  (let* ((pose (cl-tf:transform->pose  (remove-z
                                        ;; removed apply-bullet-transform from here
                                        (quaternion-w-flip transform))))
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

(defun add-axes ()
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ba-axes ((1 1 1) (0 0 0 1))
                            :mass 0.2 :color (0 1 1) :mesh :ba-axes)))))


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
    (move-object (flip-left-to-right-handedness (make-poses "?PoseHandStart")) 'ba-axes3)))
