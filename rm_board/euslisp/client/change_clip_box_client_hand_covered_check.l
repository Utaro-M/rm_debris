#!/usr/bin/env roseus
(ros::load-ros-manifest "rm_debris_vision")
(ros::roseus "ochange_hand_covered_check_area_client")
(ros::wait-for-service "clip_covered_area")
;;right left
(defun send_box_size (&key (box-position-list (list #f(100 100 100) #f(100 100 100) )) (size-vector-list (list #f(1 1 1) #f(1 1 1))) (tilt-vector-list (list #f(1 1 1) #f(1 1 1))))

  (setq req (instance rm_debris_vision::Clip_Point_CoverCheckRequest :init))
  (if (v= (car box-position-list) #f(100 100 100))
      (send req :use_auto_clip_point :data t)
    (send req :use_auto_clip_point :data nil))

    (setq box-position (elt box-position-list 0)
          size-vector (elt size-vector-list 0)
          tilt-vector (elt tilt-vector-list 0))

    (send req :box_size_r :x (elt size-vector 0))
    (send req :box_size_r :y (elt size-vector 1))
    (send req :box_size_r :z (elt size-vector 2))

    (send req :box_tilt_r :x (rad2deg (elt tilt-vector 0)))
    (send req :box_tilt_r :y (rad2deg (elt tilt-vector 1)))
    (send req :box_tilt_r :z (rad2deg (elt tilt-vector 2)))

    (send req :clip_point_req_r :x (* 0.001 (elt box-position 0)))
    (send req :clip_point_req_r :y (* 0.001 (elt box-position 1)))
    (send req :clip_point_req_r :z (* 0.001 (elt box-position 2)))
    
    ;; (format t "clip box center point = (~A ,~A ,~A)~%" (elt box-position 0) (elt box-position 1) (elt box-position 2))

    
    (setq box-position (elt box-position-list 1)
          size-vector (elt size-vector-list 1)
          tilt-vector (elt tilt-vector-list 1))

    (send req :box_size_l :x (elt size-vector 0))
    (send req :box_size_l :y (elt size-vector 1))
    (send req :box_size_l :z (elt size-vector 2))

    (send req :box_tilt_l :x (rad2deg (elt tilt-vector 0)))
    (send req :box_tilt_l :y (rad2deg (elt tilt-vector 1)))
    (send req :box_tilt_l :z (rad2deg (elt tilt-vector 2)))

    (send req :clip_point_req_l :x (* 0.001 (elt box-position 0)))
    (send req :clip_point_req_l :y (* 0.001 (elt box-position 1)))
    (send req :clip_point_req_l :z (* 0.001 (elt box-position 2)))

    ;; (format t "clip box size         = (~A ,~A ,~A)~%" (elt size-vector 0) (elt size-vector 1) (elt size-vector 2))
    ;; (format t "clip box tilt         = (~A ,~A ,~A)~%" (elt tilt-vector 0) (elt tilt-vector 1) (elt tilt-vector 2))  
    ;; (format t "clip box center point = (~A ,~A ,~A)~%" (elt box-position 0) (elt box-position 1) (elt box-position 2))
    ;; (format t "clip box center point = (~A ,~A ,~A)~%" (send res :clip_point_r :x) (send res :clip_point_r :y) (send res :clip_point_ :z))
    (setq res (ros::service-call "clip_covered_area" req t))
)
;; (send_box_size 1.0 0.1 1 1 1 1)

(defun check-hand-cover()
  (let* ((line-vec-l (v- (v- (send (send *robot* :larm :end-coords) :worldpos) (send *robot* :lleg :end-coords :worldpos)) (v- (send (car (send *robot* :cameras)) :worldpos) (send *robot* :lleg :end-coords :worldpos))))
         (line-vec-r (v- (v- (send (send *robot* :rarm :end-coords) :worldpos) (send *robot* :lleg :end-coords :worldpos)) (v- (send (car (send *robot* :cameras)) :worldpos) (send *robot* :lleg :end-coords :worldpos))))
         
         (line-l (make-line (v- (send (car (send *robot* :cameras)) :worldpos) (send *robot* :lleg :end-coords :worldpos)) (v- (send (send *robot* :larm :end-coords) :worldpos) (send *robot* :lleg :end-coords :worldpos))))
         (line-r (make-line (v- (send (car (send *robot* :cameras)) :worldpos) (send *robot* :lleg :end-coords :worldpos)) (v- (send (send *robot* :rarm :end-coords) :worldpos) (send *robot* :lleg :end-coords :worldpos))))

         (prism-l (make-prism '( #f(10 0 0) #f(10 10 0) #f(0 10 0) #f(0 0 0)) line-vec-l))
         (prism-r (make-prism '( #f(10 0 0) #f(10 10 0) #f(0 10 0) #f(0 0 0)) line-vec-r)))
    ;; (send pri :locate (v- (send (car (send *robot* :cameras)) :worldpos) (send *robot* :lleg :end-coords :worldpos)))
    (send prism-l :locate (send (car (send *robot* :cameras)) :worldpos))
    (send prism-r :locate (send (car (send *robot* :cameras)) :worldpos))
    (send prism-l :set-color #f(0 0 1))
    (send prism-r :set-color #f(0 0 1))
    (objects (list prism-l prism-r *robot*))

    (setq rot-l (transpose (matrix (normalize-vector line-vec-l) (normalize-vector (send line-l :point (send line-l :foot #f(0 0 0)))) (normalize-vector (v* (normalize-vector line-vec-l) (normalize-vector (send line-l :point (send line-l :foot #f(0 0 0)))))))))
    (setq rot-r (transpose (matrix (normalize-vector line-vec-r) (normalize-vector (send line-r :point (send line-r :foot #f(0 0 0)))) (normalize-vector (v* (normalize-vector line-vec-r) (normalize-vector (send line-r :point (send line-r :foot #f(0 0 0)))))))))

    (setq box-coords-l (make-coords :pos (send line-l :point 0.5) :rot rot-l))
    (setq box-coords-r (make-coords :pos (send line-r :point 0.5) :rot rot-r))

    (setq *box-coords-l* (instance arrow-object :init))
    (send *box-coords-l* :newcoords box-coords-l)
    (setq *box-coords-r* (instance arrow-object :init))
    (send *box-coords-r* :newcoords box-coords-r)
    (objects (list prism-l prism-r *robot* *box-coords-l* *box-coords-r*))
    (print "right")
    (format t "position r=~A~%" (send *box-coords-r* :worldpos))
    (format t "size-vetor =~A~%" (float-vector 0.3 0.3 (- (* 0.001 (send line-r :length)) 0.2)))
    (format t "tilt-vetor =~A~%" (elt (rpy-angle rot-r) 0))
    (print "left")    
    (format t "position l=~A~%" (send *box-coords-l* :worldpos))
    (format t "size-vetor =~A~%" (float-vector 0.3 0.3 (- (* 0.001 (send line-l :length)) 0.2)))
    (format t "tilt-vetor =~A~%" (elt (rpy-angle rot-l) 0))

    (send_box_size :box-position-list (list (send *box-coords-r* :worldpos) (send *box-coords-l* :worldpos)) :size-vector-list (list (float-vector (- (* 0.001 (send line-r :length)) 0.4) 0.05 0.05 ) (float-vector (- (* 0.001 (send line-l :length)) 0.4) 0.05 0.05 )) :tilt-vector-list (list (elt (rpy-angle rot-r) 0) (elt (rpy-angle rot-l) 0)))
    )
  )
