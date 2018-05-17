(in-package :le)
;; usefull bullet world infos

;; get-info of an object
(defun get-info (infoObj)
  (cut:var-value (intern infoObj)  *poses-list*))

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
                    (human-to-robot-hand-transform)
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
  (human-to-robot-hand-transform)
  )






;; look up transfrom from tf. ex: "l_wrist_roll_link" "l_gripper_l_finger_tip_link" 
(defun lookup-tf-transform (parent_frame child_frame)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
    (cram-tf::lookup-transform cram-tf::*transformer* parent_frame child_frame)))
;;------------------------------------------------------------------------------------------
;; plans

(defun alternative-demo (object)
  (move-object  (make-poses "?PoseObjStart") object)
  ;; (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
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
;; new now! maybe rename...
(defun apply-bullet-transform (transform)
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector -2.65 -0.7 0.0)
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
  (add-fork)
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

;; for spawning boxes on the edges of the table
;; or generally to be used when more then one object of one kind needs to be spawned somewhere. 
(defun table-calibration (max)
  (let (temp-name)
    (get-grasp-something-poses)
    (dotimes (c max )
      (get-next-obj-poses c)
      (setf temp-name (intern (concatenate 'string  "ba-muesli" (write-to-string c))))
      (add-muesli temp-name)
      (format nil "added: ~D out of: ~D ~%" c max)
      (move-object (make-poses "?PoseCameraStart") temp-name)))
)


;;; evaluation
(defun move-obj-with-offset (x-offset y-offset object)
(move-object 
     (cl-tf:make-transform 
      (cl-tf:make-3d-vector
       (+
        x-offset
        (cl-tf:x
         (cl-tf:translation
          (make-poses "?PoseObjStart"))))
       (+
        y-offset
        (cl-tf:y
         (cl-tf:translation
          (make-poses "?PoseObjStart"))))
       (cl-tf:z
        (cl-tf:translation 
         (make-poses "?PoseObjStart"))))
      (cl-tf:rotation
       (make-poses "?PoseObjStart"))) object))
;;--------------------------------------------------------------
(defun demo-all-obj ()
  (get-grasp-something-poses)
  ;; muesli
  (alternative-demo 'ba-muesli)
  (execute-pick-and-place :ba-muesli)

  ;; milk
  (get-next-obj-poses 1)
  (alternative-demo 'ba-milk)
  (execute-pick-and-place :ba-milk)

  ;; cup
  (get-next-obj-poses 2)
  (alternative-demo 'ba-cup)
  (execute-pick-and-place :ba-cup)

  ;; bowl
  (get-next-obj-poses 3)
  (alternative-demo 'ba-bowl)
  (execute-pick-and-place :ba-bowl)

  ;; fork
  (get-next-obj-poses 4)
  (alternative-demo 'ba-fork)
  (execute-pick-and-place :ba-fork)
  )

(defun demo-spawn-obj-in-place ()
  (get-grasp-something-poses)
  ;; muesli
  (alternative-demo 'ba-muesli)
  
  ;; milk
  (get-next-obj-poses 1)
  (alternative-demo 'ba-milk)

  ;; cup
  (get-next-obj-poses 2)
  (alternative-demo 'ba-cup)

  ;; bowl
  (get-next-obj-poses 3)
  (alternative-demo 'ba-bowl)

  ;; fork
  (get-next-obj-poses 4)
  (alternative-demo 'ba-fork))

(defun demo-all-pick-place ()
  (get-next-obj-poses 0)
  (execute-pick-and-place :ba-muesli)
  (get-next-obj-poses 1)
  
  (execute-pick-and-place :ba-milk)
  (get-next-obj-poses 2)
  (execute-pick-and-place :ba-cup)
  (get-next-obj-poses 3)
  (execute-pick-and-place :ba-bowl)
  (get-next-obj-poses 4)
  (execute-pick-and-place :ba-fork))


(defun spawn-without-transform ()
  (get-next-obj-poses 0)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-muesli)
  (get-next-obj-poses 1)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-milk)
  (get-next-obj-poses 2)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-cup)
  (get-next-obj-poses 3)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-bowl)
  (get-next-obj-poses 4)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-fork)

  (move-to-object (set-grasp-base-pose (make-poses-without-transform "?PoseCameraStart")) (set-grasp-look-pose (make-poses-without-transform "?PoseObjStart"))))
  

(defun make-poses-without-transform (name &optional (poses-list *poses-list*))
  (make-pose (cut:var-value (intern name) poses-list)))

;;-------------------------
(defun make-poses-with-quaternion (name &optional (poses-list *poses-list*))
  (quaternion-w-flip
   (make-pose (cut:var-value (intern name) poses-list))))
(defun spawn-with-quaternion ()
  (get-next-obj-poses 0)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-muesli)
  (get-next-obj-poses 1)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-milk)
  (get-next-obj-poses 2)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-cup)
  (get-next-obj-poses 3)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-bowl)
  (get-next-obj-poses 4)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-fork)

  (move-to-object (set-grasp-base-pose (make-poses-with-quaternion "?PoseCameraStart")) (set-grasp-look-pose (make-poses-with-quaternion "?PoseObjStart"))))


;; move to utils?----------------------------------------------------------------------------------
(defun move-head (pose)
  (prolog:prolog `(and (btr:bullet-world ?world)
                       (cram-robot-interfaces:robot ?robot )
                       (btr:head-pointing-at ?world ?robot ,pose))))

;;if in back 'cereal-5
(defun is-in-view (name-of-object)
  (prolog:prolog `(and (btr:bullet-world ?world)
                              (cram-robot-interfaces:robot ?robot)
                              (btr:visible ?world ?robot ,name-of-object))))
;; move to util end--------------------------------------------------------------------------------




; moves the given object to the given pose.
; usage example: (move-object (pose-lists-parser '|?PoseObjEnd|))
(defun move-object (transform obj)
  (let* ((pose (cl-tf:transform->pose transform)))
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world ,obj ,pose))))))








;;(make-poses "?PoseCameraStart")
(defun move-robot (transform)
  ;; make the transform a viable robot position
  (let* ((pose (cl-tf:transform->pose  (remove-z transform )))
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




;;---------------------------
;; functions which are not called by anyone anymore and might just get deleted: 
(defun place-offset-transform ()
  (let ()
      (cl-tf:make-transform
       (cl-tf:make-3d-vector 0.0 0.2 0.0)
       (cl-tf:make-identity-rotation))))

;parses the output from knowrob to a proper string which prolog can use
(defun parse-str (str)
  (concatenate 'string "'"  (remove #\' (string str)) "'"))
