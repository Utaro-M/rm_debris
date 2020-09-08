(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")



;;(load "package://hrpsys_ros_bridge_wrs/euslisp/jaxon_red-interface.l")
(load "/home/utaro/catkin_ws/jaxon_tutorial/src/rtmros_tutorials/hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)
;; (send *ri* :srart-auto-balancer)

;;plot
(send *ri* :set-log-maxlength (* 30 500))
(load "package://control_tools/play-hrpsysfile.l")

;; (setq target (instance arrow-object :init))
;; (send target :newcoords
;;         (make-coords :pos (float-vector 0 0 0) :rpy (list 0 0 0)))
;; (send *robot* :larm :inverse-kinematics target  :move-target (send *robot* :rarm-end-coords) :rotation-axis :z );;:debug-view t)

;; send *robot* :rarm :move-end-pos #f(100 0 0)

(defun grab ()
(setq *rarm-end* (send *robot* :rarm :end-coords :copy-worldcoords))
;; (send *rarm-end* :translate (float-vector 140.0 0.0 60.0))
;; (send *rarm-end* :rotate (/ pi 4) :y)
(send *rarm-end* :translate (float-vector 200.0 0.0 20.0))
(send *rarm-end* :rotate (/ pi 2) :y)
(send *robot* :rarm :inverse-kinematics *rarm-end* :rotation-axis :z)

(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)
(send *robot* :rarm :move-end-pos #f(140 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp)

(snap 0 (/ -pi 2) 0)

(objects (list *robot* (send *robot* :rarm :end-coords :copy-worldcoords)))
)

(defun init ()
  (send *robot* :reset-manip-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

    )

(defun shake (x y z)
  (send *robot* :rarm :move-end-pos (float-vector x y z))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun snap (x y z)
  (setq *rarm-end* (send *robot* :rarm :end-coords :copy-worldcoords))
  (if x
      (send *rarm-end* :rotate x :x)
    )
  (if y
      (send *rarm-end* :rotate y :y)
    )
  (if z
      (send *rarm-end* :rotate z :z)
    )
  
  (send *robot* :rarm :inverse-kinematics *rarm-end* :rotation-axis :z)

  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)


  )

(defun demo ()
  
  (snap 0 (/ (* 2 pi) 3) 0)
  (snap 0 (/ (* -2 pi) 3) 0)
  ;; (snap (/ pi 3) 0 0)
  ;; (snap (/ -pi 3) 0 0)  
    )

(defun log ()
  
  (save-log :fname "stick_contact")
  )

(defun series()
  (init)
  (grab)
  
  (send *ri* :start-log )
  ;;(snap 0 (/ (* 1 pi) 3) 0)
  (unix:usleep 3000)
  (save-log :fname "stick_contact_i")
  (unix:usleep 1000)
  
  (send *ri* :start-log )
  (snap 0 (/ (* 1 pi) 2) 0)
  (unix:usleep 3000)
  (save-log :fname "stick_contact_f")
  (unix:usleep 1000)
  
  (snap 0 (/ (* -1 pi) 2) 0)

  (send *ri* :start-log )
  (snap (/ (* -1 pi) 2) 0  0)
  (unix:usleep 3000)
  (save-log :fname "stick_contact_l")
  (unix:usleep 1000)

  (snap (/ (* 1 pi) 2) 0  0)  

  (send *ri* :start-log )
  (snap (/ (* 1 pi) 2) 0  0)
  (unix:usleep 3000)
  (save-log :fname "stick_contact_r")
  (unix:usleep 1000)    
  
  )
