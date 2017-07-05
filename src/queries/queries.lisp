(in-package :ques)

;;Integreation of the queries which are already predefined in ease.

(defun init-clean-table (&optional (node NIL))
  (if node (start-ros-node "lisp_ease"))
  (print "start initializing...")
  (print "start a ros node for communication")
  (start-ros-node "lisp_ease")
  (prolog-simple "register_ros_package(knowrob_robcog).")
  (prolog-simple "u_load_episodes('/media/hasu/Exte/episodes/Virtual-Games/clean-table/rcg_0/Episodes/').")
  (prolog-simple "owl_parse('/media/hasu/Exte/episodes/Virtual-Games/clean-table/rcg_0/SemanticMap.owl').")
  (prolog-simple "connect_to_db('Virtual-Games_clean-table').")
  (prolog-simple "sem_map_inst(MapInst),!,marker_update(object(MapInst)).")
  (print "done initializing."))


(defun init-set-table (&optional (node NIL))
  (if node (start-ros-node "lisp_ease"))
(prolog-simple "register_ros_package(knowrob_robcog),
 u_load_episodes('/media/hasu/Exte/episodes/Virtual-Games/table-set/rcg_5/Episodes/'),
 owl_parse('/media/hasu/Exte/episodes/Virtual-Games/table-set/rcg_5/SemanticMap.owl'),
 connect_to_db('Virtual-Games_table-set'),
 sem_map_inst(MapInst),!,
 marker_update(object(MapInst))."))






;;ct: clean table, st: set table
(defun ct-grasp-something-pose ()
  (print "Hand, head and object pose at beginning of grasp.")
  (prolog-simple "ep_inst(EpInst),
    u_occurs(EpInst, EventInst, Start, End),
    event_type(EventInst, knowrob:'GraspingSomething'),
    rdf_has(EventInst, knowrob:'objectActedOn',ObjActedOnInst),
    u_marker_remove_all,performed_by(EventInst, HandInst),
    iri_xml_namespace(HandInst,_, HandInstShortName),
    obj_type(HandInst, HandType),
    iri_xml_namespace(HandType, _, HandTypeName),
    atom_concat('package://sim/unreal/', HandTypeName, _TempPath),
    atom_concat(_TempPath, '/', HandModelPath),
    view_bones_meshes(EpInst, HandInstShortName, Start, HandModelPath),
    iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
    actor_pose(EpInst, ObjShortName, Start, Pose),
    u_split_pose(Pose, Pos, Quat),
    marker_pose(object(ObjActedOnInst),
    pose(Pos,Quat)),obj_type(CameraInst, knowrob:'CharacterCamera'),
    iri_xml_namespace(CameraInst, _, CameraShortName),
    actor_pose(EpInst, CameraShortName, Start, PoseC),
    u_split_pose(PoseC, PosC, QuatC),
    marker_pose(object(CameraInst), pose(PosC,QuatC))."))



(defun st-grasp-spmething-pose ()
  (prolog-simple "
   ep_inst(EpInst),
   u_occurs(EpInst, EventInst, Start, End),
   event_type(EventInst, knowrob:'GraspingSomething'), 
   rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
   u_marker_remove_all,
   performed_by(EventInst, HandInst),
   iri_xml_namespace(HandInst,_, HandInstShortName),
   obj_type(HandInst, HandType),
   iri_xml_namespace(HandType, _, HandTypeName),
   atom_concat('package://sim/unreal/', HandTypeName, _TempPath), atom_concat(_TempPath, '/', HandModelPath),
   view_bones_meshes(EpInst, HandInstShortName, Start, HandModelPath),
   iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
   actor_pose(EpInst, ObjShortName, Start, Pose),
   u_split_pose(Pose, Pos, Quat),
   marker_pose(object(ObjActedOnInst), pose(Pos,Quat)),
   obj_type(CameraInst, knowrob:'CharacterCamera'),
   iri_xml_namespace(CameraInst, _, CameraShortName),
   actor_pose(EpInst, CameraShortName, Start, PoseC),
   u_split_pose(PoseC, PosC, QuatC),
   marker_pose(object(CameraInst), pose(PosC,QuatC))."))


(defun ct-release-object-with-trajectory ()
  (prolog-simple "ep_inst(EpInst),
   u_occurs(EpInst, EventInst, Start, End),
   event_type(EventInst, knowrob:'GraspingSomething'), 
   rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
   u_marker_remove_all,
   performed_by(EventInst, HandInst),
   iri_xml_namespace(HandInst,_, HandInstShortName),
   obj_type(HandInst, HandType),
   iri_xml_namespace(HandType, _, HandTypeName),
   atom_concat('package://sim/unreal/', HandTypeName, _TempPath), atom_concat(_TempPath, '/', HandModelPath),
   view_bones_meshes(EpInst, HandInstShortName, End, HandModelPath),
   iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
   view_actor_traj(EpInst, ObjShortName, Start, End, 'point', 'green', 0.03, 0.01),
   actor_pose(EpInst, ObjShortName, End, Pose),
   u_split_pose(Pose, Pos, Quat),
   marker_pose(object(ObjActedOnInst), pose(Pos,Quat)),
   obj_type(CameraInst, knowrob:'CharacterCamera'),
   iri_xml_namespace(CameraInst, _, CameraShortName),
   actor_pose(EpInst, CameraShortName, End, PoseC),
   u_split_pose(PoseC, PosC, QuatC),
   marker_pose(object(CameraInst), pose(PosC,QuatC))."))


(defun st-release-object-with-trajectory ()
  (prolog-simple "ep_inst(EpInst),
   u_occurs(EpInst, EventInst, Start, End),
   event_type(EventInst, knowrob:'GraspingSomething'), 
   rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
   u_marker_remove_all,
   performed_by(EventInst, HandInst),
   iri_xml_namespace(HandInst,_, HandInstShortName),
   obj_type(HandInst, HandType),
   iri_xml_namespace(HandType, _, HandTypeName),
   atom_concat('package://sim/unreal/', HandTypeName, _TempPath), atom_concat(_TempPath, '/', HandModelPath),
   view_bones_meshes(EpInst, HandInstShortName, End, HandModelPath),
   iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
   view_actor_traj(EpInst, ObjShortName, Start, End, 'point', 'green', 0.03, 0.01),
   actor_pose(EpInst, ObjShortName, End, Pose),
   u_split_pose(Pose, Pos, Quat),
   marker_pose(object(ObjActedOnInst), pose(Pos,Quat)),
   obj_type(CameraInst, knowrob:'CharacterCamera'),
   iri_xml_namespace(CameraInst, _, CameraShortName),
   actor_pose(EpInst, CameraShortName, End, PoseC),
   u_split_pose(PoseC, PosC, QuatC),
   marker_pose(object(CameraInst), pose(PosC,QuatC))."))

(defun test ()
  (prolog-simple "ep_inst(Ep_Inst)"))


