(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)

(defun prepare-real-machine(&key (set-ref-force-updater? t) (emergency-off? t))
  (send *ri* :start-auto-balancer :limbs (list :lleg :rleg :larm :rarm))
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 200 :k-r 200)
  (if set-ref-force-updater?
      (progn 
	(send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 0.5 :frame "world")
	(send *ri* :start-reference-force-updater :arms)
	(print "set ref force updater ON")))
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (setq flag-simulator nil)
  (if emergency-off?
      (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
	(warn "disable emergency-check~%")
	;;COPのemergencyをきる
	(send *ri* :stop-st)
	(send *ri* :set-st-param :emergency-check-mode 0)
	(send *ri* :start-st)))
  (send *ri* :set-log-maxlength (* 500 120))
  (format t "prepare-real-machine~%")
  )

;;脚を開いた状態から0.1m前進する
(defun debag-func()
  (prepare-real-machine)
  (send *robot* :reset-manip-pose)
  (send *robot* :move-coords (make-coords) (send *robot* :lleg :end-coords))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  (setq lleg (make-coords :pos #f(15.0 160.0 1.705e-13) :rpy #f(-3.466e-18 1.546e-19 1.388e-17)))
  (setq rleg (make-coords :pos #f(-14.787 -158.721 0.002) :rpy #f( -0.029 -1.529e-06 -4.615e-06)))
  (send *ri* :set-foot-steps
        (list
         (make-coords :coords  (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
         (make-coords :coords lleg :name :lleg)
         (make-coords :coords  rleg :name :rleg)       
         ))

  (send *robot* :angle-vector  #f(-1.704 -6.44234 -42.038 88.5287 -47.1159 6.46033 0.05511 5.10885 -45.2001 88.979 -44.4028 -5.10899 1.13229 11.6047 -7.23663 2.69917 0.824779 37.0 26.4666 -34.0665 -4.80595 -73.4077 43.7561 -8.55578 13.5856 -37.0 25.2936 19.536 4.02469 -77.5973 -48.1484 12.0398 25.3355))
  (send *robot* :move-coords (make-coords) (send *robot* :lleg :end-coords))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  
  (setq lleg-new (make-coords :pos #f(100.0 -1.260e-13 1.816e-13) :rpy #f(-1.188e-17 -1.106e-16 6.570e-34)))
  (setq rleg-new (make-coords :pos #f(70.173 -321.571 0.002) :rpy #f( -0.029 -1.529e-06 -4.615e-06)))

  (send *ri* :set-foot-steps
        (list
         (make-coords :coords  (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
         (make-coords :coords lleg-new :name :lleg)
         (make-coords :coords  rleg-new :name :rleg)
         ))
)
