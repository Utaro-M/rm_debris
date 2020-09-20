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
  (send post-coords :draw-on :flush nil :size 300 :color #f(1 0 0))
  
  (print (format nil "post-coords = ~A" post-coords))
;;  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)

  (send pre-coords :draw-on :flush nil :size 300)
;;  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300 :color #f(0 0 1))

  (setq result (send post-coords :inverse-transformation)) ; post-coords))
  (send result :draw-on :flush nil :size 300 :color #f(0 0 1))
  (transform-coords result pre-coords ret)
  (send ret :draw-on :flush t :size 300 :color #f(0 1 0))
)
