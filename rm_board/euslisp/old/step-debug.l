(defun prepare-real-machine(&key (set-ref-force-updater? t))
  (send *ri* :start-auto-balancer :limbs (list :lleg :rleg :larm :rarm))
  (if set-ref-force-updater?
      (progn 
	(send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 0.5 :frame "world")  
	(send *ri* :start-reference-force-updater :arms)
	(print "set ref force updater ON")))
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500 :k-r 200)
  (setq flag-simulator nil)
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st))
  (send *ri* :set-log-maxlength (* 500 90))
  (setq movetime 12000)
  (format t "prepare-real-machine~%")
  )

(defun step()
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :move-gripper :larm 0.0 :wait t)
  (send *ri* :move-gripper :rarm 0.0 :wait t)
  (setq lleg-coords-new (make-coords :pos #f(48.356 245.005 -0.008) :rpy #f(0.022 4.965e-06 1.208e-05)))
  (setq rleg-coords-new (make-coords :pos #f(50 -249 -0.009) :rpy #f(-0.024 -2.481e-07 1.708e-06)))
  (setq zmp-vector (send *ri*  :zmp-vector)) 
  (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  (send *ri* :set-foot-steps
        (list
         (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
         (make-coords :coords  lleg-coords-new :name :lleg)
         (make-coords :coords  rleg-coords-new :name :rleg)
         ))
  )
