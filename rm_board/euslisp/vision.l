(load "/home/utaro/catkin_ws/jaxon_tutorial/src/rtmros_tutorials/hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(ros::roseus-add-msgs "opencv_apps")
(ros::load-ros-manifest "jsk_recognition_msgs")

(defvar *bounding-box-llist* nil)
;;colorfilter parameters

(setq path "/multisense_local/hsi_filter")
(defun param_change (x)
  (cond ((= x 0)
	 (progn
	   ;;red
	   (ros::set-dynamic-reconfigure-param path "h_limit_max" :int 20)
	   (ros::set-dynamic-reconfigure-param path "h_limit_min" :int -20)
	   (ros::set-dynamic-reconfigure-param path "s_limit_max" :int 255)
	   ;; (ros::set-dynamic-reconfigure-param path "s_limit_min" :int 80)
	   (ros::set-dynamic-reconfigure-param path "s_limit_min" :int 0)       
	   (ros::set-dynamic-reconfigure-param path "i_limit_max" :int 255)
	   ;; (ros::set-dynamic-reconfigure-param path "i_limit_min" :int 50)
	   (ros::set-dynamic-reconfigure-param path "i_limit_min" :int 0)       
       (setq str "red")

	   ))
	(t (progn
	     (ros::set-dynamic-reconfigure-param "/multisense_local/hsi_filter//euclidean_clustering" "max_size" :int 20000)
	     (ros::set-dynamic-reconfigure-param "/multisense_local/hsi_filter//euclidean_clustering" "min_size" :int 100)
	     (ros::set-dynamic-reconfigure-param path "s_limit_max" :int 25)
	     (ros::set-dynamic-reconfigure-param path "h_limit_max" :int 20)
	     (ros::set-dynamic-reconfigure-param path "h_limit_min" :int -20)
	     (ros::set-dynamic-reconfigure-param path "s_limit_max" :int 25)
	      (ros::set-dynamic-reconfigure-param path "s_limit_min" :int 0)
	      (ros::set-dynamic-reconfigure-param path "i_limit_max" :int 255)
	      (ros::set-dynamic-reconfigure-param path "i_limit_min" :int 180)
          (setq str "white")
	      )))
   (ros::ros-warn "param-change ~A" str)
   )

(setq flag t)
(defun cb_judge (msg)
  (setq handbox #f(100 100 100))
  (setq *bounding-box-list* (send msg :boxes))
  ;; (if *bounding-box-list*
  (setq vol-total-rarm 0)
  (setq vol-total-larm 0)
  (dotimes (i (length *bounding-box-list*))
    (progn (let* ((b (elt *bounding-box-list* i))
                  (base->obj-coords (ros::tf-pose->coords (send b :pose)))
                  (base-coords (send (send *robot* :body_lk) :copy-worldcoords)))
             (setq obj-pos (scale 0.001 (send (send base-coords :transform base->obj-coords) :worldpos)))
             (setq obj-ori (send (send base-coords :transform base->obj-coords) :worldrot)))
           (setq obj-size (scale 1000 (send b :dimensions)))
           (setq vol (* (- (elt obj-size 0)  (elt obj-size 1) (elt obj-size 2)))
           (setq x (elt obj-pos 0))
           (setq y (elt obj-pos 1))
           (setq z (elt obj-pos 2))
           (ros::ros-warn "x=~A,y=~A,z=~A" x y z)
           (ros::ros-warn "vol = ~A" vol)
           
           (setq rarm-coords (send *robot* :rarm :end-coords))
           (setq larm-coords (send *robot* :larm :end-coords))

           (setq rarm-pos (send rarm-coords :worldpos))
           (setq larm-pos (send larm-coords :worldpos))

           (if (< (abs (norm (v- obj-pos rarm-pos))) (/ (abs (norm handbox)) 2))
               (progn
                 (setq vol-total-rarm (+ vol vol-total))
                 )
             )
           (if (< (abs (norm (v- obj-pos larm-pos))) (/ (abs (norm handbox)) 2))
               (progn
                 (setq vol-total-larm (+ vol vol-total))
                 )
             )
           ))
  
  (setq handbox-vol (* (elt handbox 0) (elt handbox 1) (elt handbox 2)))
  (if (> (/ (* 1.0 vol-total-rarm) handbox-vol) thre-vol)
      (print "rarm holding")
    (print "rarm-drop"))
  (if (> (/ (* 1.0 vol-total-larm) handbox-vol) thre-vol)
      (print "larm holding")
    (print "larm drop"))
  )
           
  ;; (ros::ros-warn "flag~A" flag))

(ros::subscribe "multisense_local/boxes" jsk_recognition_msgs::BoundingBoxArray #'cb_go )


(defun main ()
  (init)
  (param_change 0)
  (do-until-key
   (ros::sleep)
   (ros::spin-once))
  )
