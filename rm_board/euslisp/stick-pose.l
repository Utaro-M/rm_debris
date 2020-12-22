#!/usr/bin/env roseus
;; (ros::load-ros-manifest "roseus")
(require "models/arrow-object.l")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "get_stick_pose")

(setq stick-coords nil)
(defun get-stick-pose()
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
        (setq stick-coords-rotated (send (send (send *stick-coords* :copy-worldcoords) :rotate (deg2rad 90) :y :local) :rotate (deg2rad 180) :z :local))
        (format t "stick-coords-rotated = ~A~%" stick-coords-rotated)
        (send *stick-coords-rotated* :newcoords (send stick-coords-rotated :translate #f(0 0 100) :local))
        (format t "stick_coords = ~A~%" *stick-coords*)
        (setq origin-coords (make-coords))
        (objects (list *stick-coords* *stick-coords-rotated* origin-coords))
        (setq stick_coords (send *stick-coords-rotated* :copy-worldcoords)))
    stick_coords
  )
  

;; (send *irtviewer* :draw-objects)
;; (x::window-main-one)

