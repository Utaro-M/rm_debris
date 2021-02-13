#!/usr/bin/env roseus
;; (ros::load-ros-manifest "roseus")
(require "models/arrow-object.l")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "get_stick_pose")

(setq stick-pose nil)
(defun get-stick-pose()
  (setq stick-boxes (one-shot-subscribe "/stick_pose/euclidean_clustering_decomposer/boxes" jsk_recognition_msgs::BoundingBoxArray :timeout 5000))
  (format t "stick_boxes = ~A~%" stick-boxes)
  (setq stick-coords nil)
  (if stick-boxes
      (progn
        (setq stick-pos-list (mapcar #'(lambda (v) (scale 1000 (float-vector (send v :pose :position :x) (send v :pose :position :y) (send v :pose :position :z)))) (send stick-boxes :boxes)))
        (setq stick-ori-list (mapcar #'(lambda (v) (scale 1 (float-vector (send v :pose :orientation :x) (send v :pose :orientation :y) (send v :pose :orientation :z) (send v :pose :orientation :w)))) (send stick-boxes :boxes)))
        (setq *stick-coords* (instance arrow-object :init))
        (setq *stick-coords-rotated* (instance arrow-object :init))
        (send *stick-coords* :newcoords (make-coords :pos (car stick-pos-list) :rot (quaternion2matrix (car stick-ori-list))))
        (setq stick-coords-rotated (send (send (send *stick-coords* :copy-worldcoords) :rotate (deg2rad 90) :y :local) :rotate (deg2rad 180) :z :local))
        (format t "stick-coords-rotated = ~A~%" stick-coords-rotated)
        (send *stick-coords-rotated* :newcoords (send stick-coords-rotated :translate #f(0 0 100) :local))
        (format t "stick_coords = ~A~%" *stick-coords*)
        (setq origin-coords (make-coords))
        (objects (list *stick-coords* *stick-coords-rotated* origin-coords))
        (setq stick_coords *stick-coords-rotated*)))
  (send stick_coords :copy-worldcoords)
  )
  

;; (send *irtviewer* :draw-objects)
;; (x::window-main-one)

