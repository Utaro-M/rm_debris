#!/usr/bin/env roseus
;; (ros::load-ros-manifest "roseus")
(require "models/arrow-object.l")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "get_stick_pose")

(setq stick-coords nil)
(defun get-stick-pose( &key (shift #f(0 0 0)) (rot-z 240))
  (setq stick-boxes (one-shot-subscribe "/stick_pose/euclidean_clustering_decomposer/boxes" jsk_recognition_msgs::BoundingBoxArray :timeout 5000))
  (format t "stick_boxes = ~A~%" stick-boxes)
  (setq stick-coords nil)
  (if stick-boxes
      (progn
        (setq stick-pos-list (mapcar #'(lambda (v) (scale 1000 (float-vector (send v :pose :position :x) (send v :pose :position :y) (send v :pose :position :z)))) (send stick-boxes :boxes)))
        (setq stick-ori-list (mapcar #'(lambda (v) (scale 1 (float-vector (send v :pose :orientation :x) (send v :pose :orientation :y) (send v :pose :orientation :z) (send v :pose :orientation :w)))) (send stick-boxes :boxes)))
        (setq stick-dim-list (mapcar #'(lambda (v) (scale 1000 (float-vector (send v  :dimensions :x) (send v  :dimensions :y) (send v  :dimensions :z)))) (send stick-boxes :boxes)))
        ;;より細長い物を選択
        (setq tmp 0)
        (setq idx 0)
        (dotimes (i (length stick-dim-list))
          (if (< tmp (abs (/ (elt (elt stick-dim-list i) 0) (elt (elt stick-dim-list i) 1))))
              (progn
                (setq idx i)
                (setq tmp (/ (elt (elt stick-dim-list i) 0) (elt (elt stick-dim-list i) 1))))))
        
        (setq *stick-coords* (instance arrow-object :init))
        (setq *stick-coords-rotated* (instance arrow-object :init))
        (send *stick-coords* :newcoords (make-coords :pos (elt stick-pos-list idx) :rot (quaternion2matrix (elt stick-ori-list idx))))
        ;; (setq stick-coords-rotated (send (send (send *stick-coords* :copy-worldcoords) :rotate (deg2rad 90) :y :local) :rotate (deg2rad rot-z) :z :local))
        
        (setq *rarm-coords* (instance arrow-object :init))
        (setq *stick-coords-rotated-base* (instance arrow-object :init))
        (setq stick-coords-rotated-base (send (send *stick-coords* :copy-worldcoords) :rotate (deg2rad 90) :y :local))
        (send *stick-coords-rotated-base* :newcoords stick-coords-rotated-base)
        (format t "(send stick-coords-rotated-base :z-axis) = ~A~%" (send stick-coords-rotated-base :z-axis))
        (setq dir-vec (normalize-vector (v- (send (send *stick-coords-rotated-base* :copy-worldcoords) :worldpos) (send *robot* :rarm :end-coords :worldpos))))
        ;; (setq l-omega (v* (float-vector (elt (send stick-coords-rotated-base :x-axis) 0) (elt (send stick-coords-rotated-base :x-axis) 1) 0) (float-vector (elt dir 0) (elt dir 1) 0)))
        ;; (setq l-theta (asin (/ (norm l-omega) (norm (float-vector (elt (send stick-coords-rotated-base :x-axis) 0) (elt (send stick-coords-rotated-base :x-axis) 1) 0)) (norm (float-vector (elt dir 0) (elt  dir 1) 0))))) ;;rad
        ;; (print (rad2deg l-theta))

        (setq z-vec (if (< (elt (send (send stick-coords-rotated-base :copy-worldcoords) :z-axis) 2)  0) (scale -1 (send (send stick-coords-rotated-base :copy-worldcoords) :z-axis)) (send (send stick-coords-rotated-base :copy-worldcoords) :z-axis)))
        (setq y-vec (normalize-vector (v* z-vec dir-vec)))
        (setq x-vec (normalize-vector (v* y-vec z-vec)))
        (format t "z-vec = ~A~%" z-vec)
        ;; (setq theta (asin (/ (norm omega) (norm (send stick-coords-rotated-base :z-axis)) (norm dir)))) ;;rad
                ;; (setq z-vec (if (< (elt (send (send stick-coords-rotated-base :copy-worldcoords) :z-axis) 2)  0) (scale -1 (send (send stick-coords-rotated-base :copy-worldcoords) :z-axis)) (send (send stick-coords-rotated-base :copy-worldcoords) :z-axis)))
        ;; (setq x-vec dir)
        ;; (setq z-vec (send stick-coords-rotated-base :z-axis))
        ;; (setq y-vec (normalize-vector (v* z-vec x-vec)))
        (setq rot-rarm (transpose (matrix x-vec y-vec z-vec)))


        
        (setq rarm-coords (make-coords :pos (send stick-coords-rotated-base :worldpos) :rpy (elt (rpy-angle rot-rarm)  0)))
        ;; (setq rarm-coords (make-coords :rpy (elt (rpy-angle rot-rarm)  0)))

        
        ;; (setq rarm-coords (send (send *stick-coords-rotated-base* :copy-worldcoords) :rotate l-theta :z ))
        ;; (setq rarm-coords (make-coords :pos (send *stick-coords-rotated-base* :worldpos) :rot (m* (send *stick-coords-rotated-base* :worldrot) (rotation-matrix l-theta  ))))
        (send *rarm-coords* :newcoords rarm-coords)
        ;; (format t "stick-coords-rotated = ~A~%" stick-coords-rotated)

        ;; (send *stick-coords-rotated* :newcoords (send (send stick-coords-rotated :translate (v+ #f(-40 20 200) shift) :local) :translate #f(-60 0 0) :world))
        ;;左足の位置分補正？
        (setq lleg-com-dist (v- (send *robot* :lleg :end-coords :worldpos) (midpoint 0.5 (send *robot* :lleg :end-coords :worldpos) (send *robot* :rleg :end-coords :worldpos))))
        (setq *stick-coords* (send *stick-coords* :translate (scale -1 lleg-com-dist) :world))
        ;; (setq *stick-coords-rotated* (send *stick-coords-rotated* :translate (scale -1 lleg-com-dist) :world))
        (setq *stick-coords-rotated-base* (send *stick-coords-rotated-base* :translate (scale -1 lleg-com-dist) :world))
        ;; (setq *rarm-coords* (send *rarm-coords* :translate (v+ shift (scale -1 lleg-com-dist)) :world))
        (setq *rarm-coords* (send *rarm-coords* :translate shift :world))
        (format t "stick_coords = ~A~%" *stick-coords*)
        (setq origin-coords (make-coords))
        (setq *dir-vec* (make-line #f(0 0 0) (scale 300 dir-vec) ))
        ;; (send *dir-vec* :set-color #f(1 0 0))
        (objects (list ;; *stick-coords*                       
                       ;; *stick-coords-rotated-base*
                       *rarm-coords*
                       origin-coords
                       ;; (make-line #f(0 0 0) (scale 300 x-vec))
                       (make-line #f(0 0 0) (scale 300 z-vec))
                       (make-line #f(0 0 0) (scale 500 y-vec))
                       *dir-vec*
                       ))
        ;; (setq stick_coords (send *stick-coords-rotated* :copy-worldcoords))
        ;; (setq stick_coords (send *stick-coords* :copy-worldcoords))))
        (setq stick_coords (send *rarm-coords* :copy-worldcoords))
        ))
  stick_coords
  )

;; (send *irtviewer* :draw-objects)
;; (x::window-main-one)

