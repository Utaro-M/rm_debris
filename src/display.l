#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(ros::roseus "listener")
;; method call
(defclass string-cb-class
  :super propertied-object
  :slots ())
(defmethod string-cb-class
  (:init () (ros::subscribe "off_rhsensor" geometry_msgs::WrenchStamped  #'send self :string-cb))
  (:string-cb (msg) (print (list 'cb self (send msg :force)))))
(setq m (instance string-cb-class :init))
(ros::subscribe "off_rhsensor" geometry_msgs::WrenchStamped  #'send self :string-cb))

(do-until-key
 (print "OK")
 (ros::spin-once)
 )
(defun display-arrow ()
  (do-until-key
   
   (unix:sleep 1)
   (print "arrow")
   (send (send *irtviewer* :viewer) :draw-arrow (send *robot* :rarm :end-coords :worldpos) (scale 100 (send *ri* :off-force-vector :rarm)))
   (print (scale 100 (send *ri* :off-force-vector :rarm)))
)
;;     (send (send *irtviewer* :viewer) 
   (send (send *irtviewer* :viewer) :flush )
  
  )
