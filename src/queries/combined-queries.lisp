(in-package :le)

(defvar poses-list nil)
(defvar orig-poses-list nil)

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

;; care to give the right rcg stamp
(defun init-set-clean-table ()
  (start-ros-node "lisp_ease")
  (register-ros-package "knowrob_robcog")
  (u-load-episodes "/media/hasu/Exte/episodes/Own-Episodes/set-clean-table/rcg_f/Episodes//")
  (owl-parse "/media/hasu/Exte/episodes/Own-Episodes/set-clean-table/rcg_f/SemanticMap.owl")
  (connect-to-db "Own-Episodes_set-clean-table")  
  (map-marker-init))

;parses the output from knowrob to a proper string which prolog can use
(defun parse-str (str)
  (concatenate 'string "'"  (remove #\' (string str)) "'"))

;;---

(defun get-grasp-something-poses ()

  (setq orig-poses-list (prolog-simple "ep_inst(EpInst),
    u_occurs(EpInst, EventInst, Start, End),
    event_type(EventInst, knowrob:'GraspingSomething'),
    rdf_has(EventInst, knowrob:'objectActedOn',ObjActedOnInst),
    performed_by(EventInst, HandInst),
    iri_xml_namespace(HandInst,_, HandInstShortName),
    obj_type(HandInst, HandType),

    iri_xml_namespace(HandType, _, HandTypeName),
    iri_xml_namespace(ObjActedOnInst, _, ObjShortName),

    obj_type(CameraInst, knowrob:'CharacterCamera'),
    iri_xml_namespace(CameraInst, _, CameraShortName),

    actor_pose(EpInst, ObjShortName, Start, PoseObjStart),
    actor_pose(EpInst, ObjShortName, End, PoseObjEnd),

    actor_pose(EpInst, CameraShortName, Start, PoseCameraStart),
    actor_pose(EpInst, CameraShortName, End, PoseCameraEnd),

    actor_pose(EpInst, HandInstShortName, Start, PoseHandStart),
    actor_pose(EpInst, HandInstShortName, End, PoseHandEnd)."))

  (setq poses-list (cut:lazy-car orig-poses-list)))


(defun alternative-get-grasp-something-poses ()

  (setq orig-poses-list (prolog-simple "ep_inst(EpInst),
    u_occurs(EpInst, EventInst, Start, End),
    event_type(EventInst, knowrob:'GraspingSomething'),
    rdf_has(EventInst, knowrob:'objectActedOn',ObjActedOnInst),
    performed_by(EventInst, HandInst),
    iri_xml_namespace(HandInst,_, HandInstShortName),
    obj_type(HandInst, HandType),

    iri_xml_namespace(HandType, _, HandTypeName),
    iri_xml_namespace(ObjActedOnInst, _, ObjShortName),

    obj_type(CameraInst, knowrob:'CharacterCamera'),
    iri_xml_namespace(CameraInst, _, CameraShortName),

    object_pose_at_time(ObjActedOnInst, Start, PoseObjStart),
    object_pose_at_time(ObjActedOnInst, End, PoseObjEnd),

    object_pose_at_time(CameraInst, Start, PoseCameraStart),
    object_pose_at_time(CameraInst, End, PoseCameraEnd),

    object_pose_at_time(HandInst, Start, PoseHandStart),
    object_pose_at_time(HandInst, End, PoseHandEnd)."))

  (setq poses-list (cut:lazy-car orig-poses-list)))


;; count is the number of the event we want to get poses from
(defun get-next-obj-poses (count)
  (if (setq poses-list  (cut:lazy-elt orig-poses-list count))
      (progn
        (format t "Poses from event nr. ~D ." count)
        poses-list)
      (format t "No poses for event nr. ~D available. There are no more events for this query. Query result was NIL. " count)))

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
  (quaternion-w-flip (make-pose (cut:var-value (intern name) poses-list))))





