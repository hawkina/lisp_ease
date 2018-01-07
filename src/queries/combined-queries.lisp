(in-package :le)

(defvar poses-list nil)

(defun map-marker-init ()
  (prolog-simple "sem_map_inst(MapInst),!,marker_update(object(MapInst))." ))

(defun init-clean-table ()
  (start-ros-node "lisp_ease")
  (register-ros-package "knowrob_robcog")
  (u-load-episodes "/media/hasu/Exte/episodes/Virtual-Games/clean-table/rcg_0/Episodes/")
  (owl-parse "/media/hasu/Exte/episodes/Virtual-Games/clean-table/rcg_0/SemanticMap.owl")
  (connect-to-db "Virtual-Games_clean-table")
  (map-marker-init))

(defun init-set-table ()
  (start-ros-node "lisp_ease")
  (register-ros-package "knowrob_robcog")
  (u-load-episodes "/media/hasu/Exte/episodes/Virtual-Games/table-set/rcg_5/Episodes/")
  (owl-parse "/media/hasu/Exte/episodes/Virtual-Games/table-set/rcg_5/SemanticMap.owl")
  (connect-to-db "Virtual-Games_table-set")
  (map-marker-init))

(defun get-event ()
  (u-occurs "EpInst" "EventInst" "Start" "End"))

(defun get-event-by-type (type)
  (cut:lazy-append
   (event-type "EventInst" (concatenate 'string "knowrob:'" type "'"))
   (event-type "EventInst" (concatenate 'string "knowrob_u:'" type "'"))))

(defun get-all-events (&optional (type NIL))
  (if type
      (cut:force-ll (get-event-by-type type))
      (cut:force-ll (get-event))))

;parses the output from knowrob to a proper string which prolog can use
(defun parse-str (str)
  (concatenate 'string "'"  (remove #\' (string str)) "'"))


(defun get-poses-from-event ()
  (let ((episode-instance)
        (event-inst)
        (hand-inst)
        (hand-type)
        (obj-acted-on-inst)
        (hand-inst-short)
        (start)
        (end)
        (temp-list)
       ; (poses-list)
        (camera-inst)
        (camera-short)
        (obj-short))
    (progn
      (setq episode-instance (parse-str (ep-inst "EpInst")))
      (setq event-inst (parse-str (cut:var-value '|?EventInst| (car (event-type "EventInst" "knowrob:'GraspingSomething'")))))
      (setq temp-list (u-occurs episode-instance event-inst "Start" "End"))
      (setq start (parse-str (cut:var-value '|?Start| (car temp-list))))
      (setq end (parse-str (cut:var-value '|?End| (car temp-list))))
      (setq obj-acted-on-inst (parse-str (cut:var-value '|?ObjActedOnInst| (car (rdf-has event-inst "knowrob:'objectActedOn'" "ObjActedOnInst")))))
      (setq hand-inst (parse-str (cut:var-value '|?HandInst| (car (performed-by event-inst "HandInst")))))
      (setq hand-inst-short (parse-str (cut:var-value '|?HandInstShortName| (car (iri-xml-namespace hand-inst  "HandInstShortName")))))
      (setq hand-type (parse-str (cut:var-value '|?HandType| (car (obj-type "HandInst" "HandType")))))
      (iri-xml-namespace hand-type "HandTypeName")
      (setq obj-short (parse-str (cut:var-value '|?ObjShortName| (car (iri-xml-namespace obj-acted-on-inst  "ObjShortName")))))
      (setq camera-inst (parse-str (cut:var-value '|?CameraInst| (car (obj-type "CameraInst" "knowrob:'CharacterCamera'")))))
      (setq camera-short (parse-str (cut:var-value '|?CameraShortName| (car (iri-xml-namespace camera-inst "CameraShortName")))))
      (push (caar (actor-pose episode-instance hand-inst-short start "PoseHandStart")) poses-list)
      (push (caar (actor-pose episode-instance hand-inst-short end "PoseHandEnd")) poses-list)
      (push (caar (actor-pose episode-instance camera-short start "PoseCameraStart")) poses-list)
      (push (caar (actor-pose episode-instance camera-short end "PoseCameraEnd")) poses-list)
      (push (caar (actor-pose episode-instance obj-short start "PoseObjStart")) poses-list)
      (push (caar (actor-pose episode-instance obj-short end "PoseObjEnd")) poses-list)
      )))

(defun make-time-stamped-pose (pose)
  (cl-transforms-stamped:make-pose-stamped "map" (roslisp:ros-time)
                                           (cl-transforms:make-3d-vector (first pose) (second pose) (third pose))
                                           (cl-transforms:make-quaternion (fourth pose) (fifth pose) (sixth pose) (rest pose))))

(defun make-time-stamped-poses ()
  (make-time-stamped-pose (cut:var-value '|?PoseHandStart| (get-poses-from-event))))


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
