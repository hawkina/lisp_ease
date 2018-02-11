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
  (let* ((episode-instance)
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
      ;;clean poses list before unexpected stuff happenes. 
      (setq poses-list nil)
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
      (push (list '|?HandInstShortName| hand-inst-short) poses-list))))







;; there must be a prettier way of doing this...?
;; gets a list of 7 values as a parameter and makes a cl-transform out of it.
(defun make-pose (pose)
  (cl-tf:make-transform
   (apply #'cl-tf:make-3d-vector (subseq pose 0 3))
   (apply #'cl-tf:make-quaternion (subseq pose 3 7))))

;; makes a pose of an object and given time. Gets the name of the object as param
;; Note: the name has to be as the event-get-all-values function knows it
;; example: "?PoseHandStart"
(defun make-poses (name)
  (make-pose (cut:var-value (intern name) poses-list)))

;; for getting infos out of the event data which are not poses
;; ?var is the variable name we want to get infos about
(defun get-info (?var)
  (cut:var-value (intern ?var) poses-list))

;; returns the hand used in the curretnly loaded episode
(defun get-hand ()
  (if (search "Left" (car (get-info "?HandInstShortName")))
      :left
      (if (search "Right" (car (get-info "?HandInstShortName")))
          :right
          NIL)))


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


;----------------------------
(defun init-test ()
  (start-ros-node "lisp_ease")
  (register-ros-package "knowrob_robcog")
  (u-load-episodes "/media/hasu/Exte/episodes/Virtual-Games/own-setup/rcg_0/Episodes/")
  (owl-parse "/media/hasu/Exte/episodes/Virtual-Games/own-setup/rcg_0/SemanticMap.owl")
  (connect-to-db "Virtual-Games_own-setup")
  (map-marker-init))
