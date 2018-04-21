(in-package :le)

(defparameter *mesh-files*
  '((:ba-bowl "package://lisp_ease/resource/ba_bowl.stl" t)
    (:ba-cup "package://lisp_ease/resource/ba_cup.stl" t)
    (:ba-muesli "package://lisp_ease/resource/ba_muesli.stl" t)
    (:ba-fork "package://lisp_ease/resource/ba_fork.stl" t)
    (:ba-milk "package://lisp_ease/resource/ba_milk.stl" t)
    (:ba-axes "package://lisp_ease/resource/ba_axes.stl" t)))

; appends the newly defined objects in the *mesh-files* variable to the *mesh-files*
; of btr, so that they can be loaded
(defun append-meshes-to-list ()
     (btr:add-objects-to-mesh-list "lisp_ease"))
