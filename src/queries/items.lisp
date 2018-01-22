(in-package :le)

(defparameter *mesh-files*
  '((:edeka-red-bowl "package://lisp_ease/resource/edeka_red_bowl.stl" t)))


;;btr: add-object?

;(defclass item (object)
;  ((types :reader item-types :initarg :types)))

;(defmethod add-object ((world bt-world) (type (eql :edeka-red-bowl)) name pose
;                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
;  (assert size)
;  (btr::make-item world name (list type)
;             (list
;              (make-instance 'rigid-body
;                :name name :mass mass :pose (btr::ensure-pose pose)
;                :collision-shape (make-instance 'colored-box-shape ;might also be 'colored-sphere-shape
;                                   :half-extents (btr::ensure-vector size)
;                                   :color color)))))


(defun append-meshes-to-list ()
  ;(append btr::*mesh-files* '((:edeka-red-bowl "package://lisp_ease/resource/edeka_red_bowl_stl" T)))
  (btr:add-objects-to-mesh-list "lisp_ease"))
