(load "reach.l")


;;2姿勢間の手先の座標系の変換行列を求めるためのテスト
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

;; rot-coords
;; "pre-coords = #<coordinates #X557a04797290  209.447 -418.273 1127.391 / -0.157 -0.093 -0.086>"
;; Do you move real machine?(Y or N): y
;; ;; #<rotational-joint #X5579e66d3a68 CHEST_JOINT1> :joint-angle(-0.011652) violate min-angle(0.0)
;; "post-coords = #<coordinates #X557a1f5c1dd0  209.465 -418.263 1127.399 / 1.102 1.275 1.245>"
;; "change = #<coordinates #X557a1f2993a0  1389.295 -575.531 857.064 / 1.22 -1.357 -1.666>"
;; "angle = ((2.69782 1.59098 -2.92863) (5.83941 -1.59098 0.212967))"
;; "ret = #<coordinates #X557a0074b0b0  209.447 -418.273 1127.391 / -0.157 -0.093 -0.086>"
;; "ret = #<coordinates #X557a0074b0b0  209.447 -418.273 1127.391 / -0.157 -0.093 -0.086>"

(defun get-transform-matrix
    (pre-coords post-coords)
  ;;postからpreへの変換行列を作る :worldで
  (setq change (send post-coords :transformation pre-coords :world))
  (send change :draw-on :flush nil :size 300 :color #f(0 0 1))
  (print (format nil "change = ~A" change))
  change
  )

;; (setq pre-coords (send (elt *rs-list* 1000) :rarm :end-coords))
;; (setq post-coords (send (elt *rs-list* 1000) :rarm :end-coords))
