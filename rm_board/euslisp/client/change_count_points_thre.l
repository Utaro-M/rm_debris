#!/usr/bin/env roseus
(ros::load-ros-manifest "rm_debris_vision")
(ros::roseus "count_points_client")
(ros::wait-for-service "count_points_threshould")

(setq tri nil)
(defun set-cp-thre(thre_l thre_r)
  (let ((req (instance rm_debris_vision::CPthreRequest :init)))
    (send req :larm :data thre_l)
    (send req :rarm :data thre_r)
    (setq res (ros::service-call "count_points_threshould" req t))
    (format t "thre_r = ~A~%" thre_r)
    (format t "thre_l = ~A~%" thre_l)    
  )

