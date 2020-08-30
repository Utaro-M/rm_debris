(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")



;;(load "package://hrpsys_ros_bridge_wrs/euslisp/jaxon_red-interface.l")
(load "/home/utaro/catkin_ws/jaxon_tutorial/src/rtmros_tutorials/hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)

;; (setq target (instance arrow-object :init))
;; (send target :newcoords
;;         (make-coords :pos (float-vector 0 0 0) :rpy (list 0 0 0)))
;; (send *robot* :larm :inverse-kinematics target  :move-target (send *robot* :rarm-end-coords) :rotation-axis :z );;:debug-view t)

;; send *robot* :rarm :move-end-pos #f(100 0 0)

(defun grab ()
(setq *rarm-end* (send *robot* :rarm :end-coords :copy-worldcoords))
;; (send *rarm-end* :translate (float-vector 140.0 0.0 60.0))
;; (send *rarm-end* :rotate (/ pi 4) :y)
(send *rarm-end* :translate (float-vector 200.0 0.0 -20.0))
(send *rarm-end* :rotate (/ pi 2) :y)

(send *robot* :rarm :inverse-kinematics *rarm-end* :rotation-axis :z)
(send *robot* :move-end-pos #f(40 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
(objects (list *robot* (send *robot* :rarm :end-coords :copy-worldcoords)))
)

(defun init ()
  (send *robot* :reset-manip-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

    )
