(load "reach.l")

(defun rot-coords()
  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
  (setq pre-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (print (format nil "pre-coords = ~A" pre-coords))
  (send *robot* :rarm :end-coords :rot)

  (unix:sleep 1)
  (snap 0 90 0)
  (unix:sleep 1)  
  (setq post-coords (send *robot* :rarm :end-coords :copy-worldcoords))  
  (print (format nil "post-coords = ~A" post-coords))
;;  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)

  (send pre-coords :draw-on :flush nil :size 300)
  (send post-coords :draw-on :flush nil :size 300 :color #f(1 0 0))

  ;;postからpreへの変換行列を作る :worldで
  (setq change (send post-coords :transformation pre-coords :world))
  (send change :draw-on :flush nil :size 300 :color #f(0 0 1))
  (print (format nil "change = ~A" change))
  (print (format nil "angle = ~A" (send change :euler-angle)))

  (setq post (float-vector 1 1 1))
  (setq p (send post-coords :inverse-transform-vector post))
  (setq ret-vec (send (send *irtviewer* :viewer) :draw-arrow (send post-coords :worldpos) (send (send post-coords :translate (scale 100 p) ) :worldpos)))
  ;; (setq ret-vec (transform  (send change :worldrot) post))
  
  (setq ret (transform-coords change post-coords))
  (send ret :draw-on :flush t :size 300 :color #f(0 1 0))
  (print (format nil "ret = ~A" ret))
)


(defun get-transform-matrix
    (pre-coods post-coords)
  ;;postからpreへの変換行列を作る :worldで
  (setq change (send post-coords :transformation pre-coords :world))
  (send change :draw-on :flush nil :size 300 :color #f(0 0 1))
  (print (format nil "change = ~A" change))
  change
  )
