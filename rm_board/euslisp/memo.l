;;右手を下に
;;(progn (my-init-pose :move-foot? nil) (start-coords))
;;(setq rarm-coords (make-coords :pos (float-vector 500 -400 600) :rpy (float-vector (deg2rad 90) (deg2rad  -30) (deg2rad 0))))
;;(setq larm-coords (make-coords :pos (float-vector 500 400 900) :rpy (float-vector 0 (deg2rad  50) (deg2rad 90))))
;;(reach :rarm-coords rarm-coords :larm-coords larm-coords :send? nil)


;;(progn (my-init-pose :move-foot? nil) (start-coords)
;; (setq rarm-coords (make-coords :pos (float-vector 540 -500 800) :rpy (float-vector (deg2rad 100) (deg2rad  20) (deg2rad 30))))
;; (setq larm-coords (make-coords :pos (float-vector 500 400 900) :rpy (float-vector 0 (deg2rad  60) (deg2rad 90))))
;; (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t)
;; (heave #f(100 0 0) #f(0 0 0) :reset? t :fix-contact? 2 :send? nil))
