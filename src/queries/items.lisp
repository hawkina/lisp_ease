(in-package :le)

(defparameter *mesh-files*
  '((:edeka-red-bowl "package://lisp_ease/resource/edeka_red_bowl.stl" t)
    (:cup-eco-orange "package://lisp_ease/resource/cup_eco_orange.stl" t)
    (:koelln-muesli-knusper-honig-nuss "package://lisp_ease/resource/koelln_muesli_knusper_honig_nuss.stl" t)
    (:spoon-blue-plastic "package://lisp_ease/resource/spoon_blue_plastic.stl" t)
    (:weide-milch-small "package://lisp_ease/resource/weide_milch_small.stl" t)))

; appends the newly defined objects in the *mesh-files* variable to the *mesh-files*
; of btr, so that they can be loaded
(defun append-meshes-to-list ()
     (btr:add-objects-to-mesh-list "lisp_ease"))
