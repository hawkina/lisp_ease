(in-package :le)

;; --- queries important for initialization ---
(defun register-ros-package (package-name)
  (prolog-simple (concatenate 'string  "register_ros_package(" package-name ")")))

(defun u-load-episodes (episodes-path)
  (prolog-simple (concatenate 'string "u_load_episodes('" episodes-path "')")))

(defun owl-parse (semantic-map-path)
  (prolog-simple (concatenate 'string "owl_parse('" semantic-map-path "')")))

(defun connect-to-db (db-name)
  (prolog-simple (concatenate 'string "connect_to_db('" db-name "')")))

(defun sem-map-inst (map-inst &optional (stop "."))
  (prolog-simple (concatenate 'string "sem_map_inst(" map-inst ")" stop)))

(defun marker-update (obj &optional)
  (prolog-simple (concatenate 'string "marker_update(" obj ")")))

(defun object (instance)
  (prolog-simple (concatenate 'string "object(" instance ")")))

(defun prolog-cut ()
  (prolog-simple (concatenate 'string "!")))


;;--- other queries ---

(defun u-occurs (ep-inst event-inst start end)
  (prolog-simple (concatenate 'string "u_occurs(" ep-inst ", " event-inst ", " start "," end ")")))

(defun event-type (event-inst event-type)
  (prolog-simple (concatenate 'string "event_type(" event-inst ", " event-type ")")))

(defun rdf-has (event-inst obj-acted-on obj-acted-on-inst)
  (prolog-simple (concatenate 'string "rdf_has(" event-inst ", " obj-acted-on ", " obj-acted-on-inst ")")))

(defun u-marker-remove-all ()
  (prolog-simple "u_marker_remove_all"))

(defun performed-by (event-inst hand-inst)
  (prolog-simple (concatenate 'string "performed_by(" event-inst ", " hand-inst ")" )))

(defun iri-xml-namespace (obj obj-short)
  (prolog-simple (concatenate 'string "iri_xml_namespace(" obj ", _, " obj-short ")")))

(defun obj-type (obj-inst obj-type)
  (prolog-simple (concatenate 'string "obj_type(" obj-inst ", " obj-type ")")))

(defun atom-concat (a1 a2 a3)
  (prolog-simple (concatenate 'string "atom_concat(" a1 ", " a2 ", " a3 ")")))

(defun view-bone-meshes (ep-inst obj-inst-short-name start model-path)
  (prolog-simple (concatenate 'string "view_bones_meshes(" ep-inst ", " obj-inst-short-name ", " start ", " model-path ")" )))

(defun actor-pose (ep-inst obj-short-name start pose)
  (prolog-simple (concatenate 'string "actor_pose(" ep-inst ", " obj-short-name ", " start ", " pose ")")))

(defun u-split-pose (pose pos quat)
  (prolog-simple (concatenate 'string "u_split_pose(" pose ", " pos ", " quat ")")))

(defun marker-pose (obj pose)
  (prolog-simple (concatenate 'string "marker_pose(" obj ", " pose ")")))

(defun pose (pos quat)
  (prolog-simple (concatenate 'string "pose(" pos ", " quat ")")))

(defun view-actor-traj (ep-inst obj-short-name start end point color size length)
  (prolog-simple (concatenate 'string "view_actor_traj(" ep-inst ", " obj-short-name ", " start ", " end ", " point ", " color  ", " size ", " length ")")))

(defun object-pose-at-time (obj-inst start pose)
  (prolog-simple (concatenate 'string "oject_pose_at_time(" obj-inst ", " start ", " pose ")")))

(defun findall (event-inst obj event-list)
  (prolog-simple (concatenate 'string "findall(" event-inst ", " obj ", " event-list ")")))

(defun ep-inst (episode-inst)
  (string (cdaar (prolog-simple (concatenate 'string "ep_inst(" episode-inst ")")))))
