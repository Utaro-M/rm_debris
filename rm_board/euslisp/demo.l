(load "./heave.l")
(load "./vision.l")

;;(progn (my-init-pose)   (send *robot* :fix-leg-to-coords (make-coords)) (unix:sleep1) (reach) (unix:sleep 1) (dem-with-check-simulator))
;;(reach)
;;(dem-with-check-simulator)
(defun dem-with-check-simulator()

  (heave #f(0 -200 0) #f(100 200 0) :cog #f(100 0 700))

  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700) :wait? nil)
  (setq flag  (check-while-working))
  (retry-flag-checker flag)
  
  (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1))
  (retry-flag-checker flag)
  
  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700) :wait? nil)
  (setq flag  (check-while-working))
  (retry-flag-checker flag)
  
  (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1))
  (retry-flag-checker flag)
  
  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700) :wait? nil)
  (setq flag  (check-while-working))
  (retry-flag-checker flag)

  (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1))
  (retry-flag-checker flag)
  
  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700) :wait? nil)
  (setq flag  (check-while-working))
  (retry-flag-checker flag)

  )
;;   (setq  (send *robot* :angle-vector))
;;   (setq rarm-coords (send *robot* arm :end-coords))
;;   (unix:sleep 3)
;;   (send *robot* :angle-vector (send *ri* :state :angle-vector))
;;   (setq rarm-coords-drop (send *robot* arm :end-coords))
;;   (my-ik rarm-retry-coords larm-retry-coords :rleg rleg-coords :lleg lleg-coords :cog cog)
;; )

(defun demo ()
  ;; (reach #f(360 0 -700) #f(360 0 -700))
  ;; (reach #f(260 0 -700) #f(260 0 -700))
  ;; (reach #f(200 0 -710) #f(200 0 -710))
  (my-init-pose)
  ;; (send *ri* :set-impedance-controller-param :arms :D-p 400 :K-r 1e5 :D-r 1e5)
  ;; (send *ri* :start-impedance :arms :K-r 140.0 :D-r 200.0 :M-r 5.0 :K-p 300.0 :D-p 600.0 :M-p 10.0)
  
  (reach)

  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0  300 0)))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 -300 0)))
  ;; ;; (my-ik #f(0 -280 0) #f( 0 280 0))
  ;; (my-ik rarm-coords larm-coords)
  ;; ;; (send-pose 3000 :force-move t)
  ;; (send-pose 3000 :force-move nil)

  ;; (heave #f(40  -400 0) #f(40  400 0))
  ;; (heave #f(40  -300 0) #f(40  300 0))
  ;; (heave #f(60  -700 0) #f(60  700 0) :degree 10)
  (heave #f(100  -400 0) #f(100  400 0))
  (heave #f(100  -300 0) #f(100  300 0))
  ;; (setq flag-simulator nil) ;;simulatorではｔ

  ;; (heave #f(10 0 0) #f(10 0 0) :cog #f(0 0 900) )
  ;; (heave #f(40  -100 0) #f(40  100 0))

  ;; (send *ri* :start-impedance :arms :K-r 140.0 :D-r 200.0 :M-r 5.0 :K-p 340.0 :D-p 600.0 :M-p 10.0)

  ;; (send *ri* :go-pos 0.3 0 0)
  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(40  200 0)))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(40 -200 0)))
  ;; (my-ik rarm-coords larm-coords)
  ;; ;; (my-ik #f(0 -100 0) #f( 0 100 0))
  ;; ;; (send-pose 3000 :force-move t)
  ;; (send-pose 3000 :force-move nil)
  ;; (heave #f(40  200 0) #f(40  -200 0))
  
  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0  100 0)))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 -100 0)))
  ;; ;; (my-ik #f(0 -280 60) #f( 0 280 60))
  ;; ;; (send-pose 3000 :force-move t)
  ;; (send-pose 3000 :force-move nil)
  ;; (heave #f(0  100 0) #f(0  -100 0))
  
  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0  100 0)))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 -100 0)))
  ;; (send-pose 3000 :force-move nil)

  ;; (my-ik #f(0 -100 0) #f( 0 100 0))
  ;; (send-pose 3000 :force-move t)  

  )

(defun dem-simulator()
  ;; (load "~/research/rm_debris/rm_board/euslisp/heave.l")
  ;; (prepare-simulator)
  (my-init-pose)
  (reach)
  (heave #f(100 0 0) #f( 100 0 0) :cog #f(100 0 700))
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)

  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  
  (heave #f(0 -50 0) #f(0 50 0))

  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700))

  ;; (progn (setq theta 40) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))

  (heave #f(60 -400 0) #f(60 400 0) :cog #f(100 0 800))

  ;; (progn (setq theta 60) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))

  (stand-up 1000  :cog #f(100 0 900))

  (stand-up 1000  :cog #f(100 0 900))

  (heave #f(60 -100 0) #f(60 100 0) :cog #f(100 0 1000))

  ;; (progn (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))

  ;; (progn (setq theta 80) (send *ri* :set-ref-force (scale 2 (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta))))) 2000 :larm) (change-arm 140 :rarm))
  ;; (progn (setq theta 80) (send *ri* :set-ref-force (scale 2 (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta))))) 2000 :rarm) (change-arm 140 :larm))

  ;; (progn (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))

  ;; (send *ri* :go-pos 0.3 0 0)
  ;; (send *ri* :go-pos 0.3 0 0)

  ;; (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms)
  
  )

(defun dem-step-simulator()
  ;; (setq flag-simulator nil)

  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  
  (reach :fix-contact? nil)
  (heave #f(100 -200 0) #f(100 200 0) :reset? nil :fix-contact? 2)
  (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? 2)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? t)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? nil)
  ;; (setq flag-simulator t)
  )
;;実機版
(defun dem-step()
  ;; (setq flag-simulator nil)

  (my-init-pose :move-foot? nil)
  (start-coords)

  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  
  (reach :fix-contact? nil :send? t)
  (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? 2)
  (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? 2)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? t)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? nil)
  ;; (setq flag-simulator t)
  )

(defun dem-bigboard-simulator()
  ;; (setq flag-simulator nil)

  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (setq rarm-coords (make-coords :pos (float-vector 360 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 360  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))
  ;; (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  ;; ;; (reach :fix-contact? nil)
  ;; (heave #f(50 -300 0) #f(50 300 0) :reset? nil :fix-contact? 2)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? 2)

  (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  
  ;; (heave #f(-100 -900 0) #f(-100 900 0) :reset? t :fix-contact? 2 :wait? t)
  (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t)
  (change-arm 120 :rarm)
  (change-arm 120 :larm)
  ;; (heave #f(100 0 0) #f(100 0 0) :reset? t :fix-contact? 2 :wait?
  ;; t)

  ;; (send *ri* :go-pos 0.3 0 0)
  ;; (unix::sleep 1)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? 2 :wait? t)
  ;; (heave #f(100 -200 0) #f(100 200 0) :reset? t :fix-contact? 2 :wait? t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)))
  (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 100)) (send *robot* :link "LLEG_LINK5"))
  (objects)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)))
  (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 100)) (send *robot* :link "LLEG_LINK5"))
  (objects)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)))
  (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 100)) (send *robot* :link "LLEG_LINK5"))
  (objects)
  (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil :wait? t)
  ;; (heave #f(100 -200 0) #f(100 200 0) :reset? t :fix-contact? 2 :wait? t)
    ;; (heave #f(300 300 0) #f(950 -400 300) :deg-l-y -30 :deg-r-y 30 :reset? t :fix-contact? nil :wait? t :avoid-collision? nil  :rleg-pos #f(-700 -50 -10) :lleg-pos #f(-300 250 -10))
  ;; (heave #f(300 200 0) #f(400 -700 200) :deg-l-y -30 :deg-r-y 30 :reset? t :fix-contact? nil :wait? t :avoid-collision? nil  :rleg-pos #f(-700 -50 -10) :lleg-pos #f(-300 250 -10))
  (heave #f(300 200 0) #f(400 -700 200) :deg-l-y -30 :deg-r-y 30 :reset? t :fix-contact? nil :wait? t :avoid-collision? nil  :rleg-pos #f(-600 -50 -10) :lleg-pos #f(-200 250 -10))
  ;; (send *ri* :go-pos 0.2 0 0)
  ;; (heave #f(0 -400 0) #f(0 400 0) :reset? t :fix-contact? nil :wait? nil)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil :wait? nil)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil :wait? nil)
  ;; (setq flag-simulator t)

  )
(setq pre-angle 0)
;;1030
(defun dem-bigboard-pre-simulator()
  ;; (setq flag-simulator nil)

  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (send *ri* :set-gait-generator-param :default-step-time 1.2)
  (setq rarm-coords (make-coords :pos (float-vector 360 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 360  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))

  (send *ri* :set-log-maxlength (* 500 60))
  (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-hands? t)

  (setq pre-angle (send *robot* :angle-vector))
  
  ;; (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t :look-at-hands? t)
  ;; (heave #f(-110 -800 0) #f(-110 800 0) :reset? t :fix-contact? 2 :wait? t :look-at-hands? t)
  (heave #f(-80 -800 0) #f(-80 800 0) :reset? t :fix-contact? 2 :wait? t :look-at-hands? t)
  ;; (heave #f(-110 -1200 0) #f(-110 1200 0) :reset? t :fix-contact? nil :wait? t)
  ;;jikkide tamesita
  ;; (heave #f(-110 -600 0) #f(-110 600 0) :reset? t :fix-contact? 2 :wait? t)  

  )
;;1026実機実験用
(defun dem-bigboard-pre()
  ;; (setq flag-simulator nil)

  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (send *ri* :set-gait-generator-param :default-step-time 1.2)
  (setq rarm-coords (make-coords :pos (float-vector 360 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 360  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))

  (send *ri* :set-log-maxlength (* 500 60))
  (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-hands? t)

  (setq pre-angle (send *robot* :angle-vector))
  (setq movetime 20000)
  
  (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t :look-at-hands? t)

  ;; (heave #f(-110 -1200 0) #f(-110 1200 0) :reset? t :fix-contact? nil :wait? t)
  ;;jikkide tamesita
  ;; (heave #f(-110 -600 0) #f(-110 600 0) :reset? t :fix-contact? 2 :wait? t)  
  (setq movetime 16000)
  )

(defun dem-bigboard-mid()
  (change-arm 120 :rarm)
  (change-arm 120 :larm)
  
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)))
  (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 100)) (send *robot* :link "LLEG_LINK5"))
  (objects)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)))
  (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 100)) (send *robot* :link "LLEG_LINK5"))
  (objects)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)))
  (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 100)) (send *robot* :link "LLEG_LINK5"))
  (objects)
  
  (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil :wait? t)

  ;; (heave #f(300 200 0) #f(400 -700 200) :deg-l-y -30 :deg-r-y 30 :reset? t :fix-contact? nil :wait? t :avoid-collision? nil  :rleg-pos #f(-600 -50 -10) :lleg-pos #f(-200 250 -10))
  )

(defun dem-bigboard-pose-simulator()
  ;; (setq flag-simulator nil)

  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (setq rarm-coords (make-coords :pos (float-vector 340 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 340  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))
  ;; (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  ;; ;; (reach :fix-contact? nil)
  ;; (heave #f(50 -300 0) #f(50 300 0) :reset? nil :fix-contact? 2)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? 2)

  (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? nil :avoid-collision? t :apply-external-force? t)
  
  (heave #f(-80 -700 0) #f(-80 700 0) :send? nil :reset? t :fix-contact? 2 :wait? t)
  ;; (heave #f(-80 -800 0) #f(-80 800 0) :reset? t :fix-contact? 2 :wait? t)  
  (change-arm 140 :rarm :send? nil)
  (change-arm 140 :larm :send? nil)
  ;; (change-arm 160 :rarm :send? nil)
  ;; (change-arm 160 :larm :send? nil)
  (heave #f(100 -200 0) #f(100 200 0) :send? nil :reset? t :fix-contact? nil :wait? nil)
  ;; (send *ri* :go-pos 0.2 0 0)
  ;; (heave #f(0 -400 0) #f(0 400 0) :reset? t :fix-contact? nil :wait? nil)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil :wait? nil)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil :wait? nil)
  ;; (setq flag-simulator t)

  )

;;転倒テスト用
(defun dem-bigboard-step-simulator()
  ;; (setq flag-simulator nil)

  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (setq rarm-coords (make-coords :pos (float-vector 350 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 350  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))
  (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  
  (heave #f(-80 -800 0) #f(-80 800 0) :reset? t :fix-contact? 2 :wait? t)
  ;; (heave #f(0 -100 0) #f(0 100 0) :reset? t :fix-contact? nil :wait? nil)
  )

(defun dem-pre()

  (setq movetime 18000)

  (my-init-pose)
  (unix::sleep 3)
  (reach)
  (unix::sleep 3)
)

(defun dem-mid()  
  ;; (heave #f(100 0 0) #f( 100 0 0) :cog #f(100 0 700))
  ;; (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (setq movetime 9000)
  (progn (setq theta 10) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))
  (unix::sleep 3)
  
  (heave #f(50 -50 0) #f(50 50 0))
  (unix::sleep 3)
  
  (heave #f(60 -200 0) #f(60 200 0) :cog #f(100 0 700))
  (unix::sleep 3)
  
  (progn (setq theta 40) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))
  (unix::sleep 3)
  
  (heave #f(60 -400 0) #f(60 400 0) :cog #f(100 0 800))
  (unix::sleep 3)
  
  (progn (setq theta 60) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))
  (unix::sleep 3)
  
  (stand-up 1000  :cog #f(100 0 900))
  (unix::sleep 3)
  
  (stand-up 1000  :cog #f(100 0 900))
  (unix::sleep 3)
  
  ;; (stand-up 1000  :cog #f(100 0 900))
  ;; (unix::sleep 3)
  
  (heave #f(60 -100 0) #f(60 100 0) :cog #f(100 0 1000))
  (unix::sleep 3)
  
  (progn (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))
  (unix::sleep 3)

  )

(defun dem-post()
  (progn (setq theta 80) (send *ri* :set-ref-force (scale 2 (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta))))) 2000 :larm) (change-arm 140 :rarm))
  (unix::sleep 3)
  
  (progn (setq theta 80) (send *ri* :set-ref-force (scale 2 (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta))))) 2000 :rarm) (change-arm 140 :larm))
  (unix::sleep 3)
  
  (progn (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))
  )


(defun dem-with-check-pre()
  (setq movetime 18000)
  (my-init-pose)
  (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1))
  ;; (unix::sleep 3)

  (reach)
  (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1))
  ;; (unix::sleep 3)
)

(defun dem-with-check-mid()  
  ;; (heave #f(100 0 0) #f( 100 0 0) :cog #f(100 0 700))
  ;; (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (setq movetime 10000)
  
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (unix::sleep 2)

  
  ;; (heave #f(50 -50 0) #f(50 50 0))
  
  ;; (progn
  ;;   (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  ;;   (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  ;;   (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1)))

  (progn 
    (heave #f(60 -200 0) #f(60 200 0) :cog #f(100 0 700) :wait? nil)
    (setq flag  (check-while-working :thre-list (list 40 40))))

  (progn
    (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
    (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
    (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1)))

  ;; (heave #f(60 -400 0) #f(60 400 0) :cog #f(100 0 800) :wait? nil)
  (heave #f(0 -400 0) #f(0 400 0) :cog #f(100 0 800) :wait? nil)
  (setq flag  (check-while-working :thre-list (list 40 40)))

  (progn
    (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
    (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
    (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1)))

  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 800) :wait? nil)
  (setq flag  (check-while-working :thre-list (list 40 40)))

  (progn
    (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
    (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
    (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1)))

  ;; (stand-up 1000  :cog #f(100 0 900))
  ;; (setq flag  (check-while-working :thre-list (list 40 40)))

  ;; (progn
  ;;   (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  ;;   (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  ;;   (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1)))

  ;; (stand-up 1000  :cog #f(100 0 900) )
  ;; (setq flag  (check-while-working :thre-list (list 40 40)))

  ;; (progn
  ;;   (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  ;;   (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  ;;   (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1)))
  
  ;; (stand-up 1000  :cog #f(100 0 900))
  ;; (unix::sleep 3)
  
  ;; (heave #f(60 -100 0) #f(60 100 0) :cog #f(100 0 1000) :wait? nil)
  ;; (setq flag  (check-while-working :thre-list (list 40 40)))

  ;; (progn
  ;;   (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  ;;   (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  ;;   (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1)))

  )

;;1011実機実験最終
(defun dem-with-check-lift()
  (setq movetime 12000)
  (progn 
    (heave #f(60 -200 0) #f(60 200 0) :cog #f(100 0 700)))
  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700))

  (send *ri* :start-log) (unix::sleep 1)
  (heave #f(0 -400 0) #f(0 400 0) :cog #f(100 0 800) :wait? nil)
  (setq flag  (check-while-working :thre-list (list 10 10)))

  (progn
    (setq rarm-f-ref (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
    (setq larm-f-ref (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
    (setq flag (check-while-still rarm-f-ref larm-f-ref :checking-time 1)))
  (save-log :fname "check-while-lifting" :log-dir "matsuura/log")
  (seq movetime 18000)
)

(defun dem-test()
  (progn 
    (heave #f(200 -100 0) #f(200 100 0) :cog #f(100 0 700) :wait? nil)
    (setq flag  (check-while-working))))

  
;;手をしっかり回し込むバージョンと
;;斜めで横に当てて押すバージョン
(defun dem-pivot()
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  
  (my-init-pose :move-foot? nil) (start-coords)
  ;; (setq rarm-coords (make-coords :pos (float-vector 540 -500 800) :rpy (float-vector (deg2rad 100) (deg2rad  20) (deg2rad 00))))
  (setq rarm-coords (make-coords :pos (float-vector 620 -600 780) :rpy (float-vector (deg2rad 100) (deg2rad  60) (deg2rad 00))))         
  (setq larm-coords (make-coords :pos (float-vector 520 400 950) :rpy (float-vector 0 (deg2rad  60) (deg2rad 90))))

  (send *ri* :start-log)
  
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t )
  (heave #f(300 -100 400) #f(0 0 -50) :reset? t :fix-contact? 2 :send? t :apply-external-force? t)
  ;; (heave #f(100 0 0) #f(0 0 -50) :reset? t :fix-contact? 2 :send? nil)
  ;; (heave #f(50 0 200) #f(0 0 -50) :reset? t :fix-contact? 2 :send? nil)
  ;; (heave #f(100 0 100) #f(0 0 -50) :reset? t :fix-contact? nil :send? nil)
  ;; (heave #f(100 0 100) #f(0 0 -50) :reset? t :fix-contact? nil :send? nil)
  )

(defun dive-board()
  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (setq rarm-coords (make-coords :pos (float-vector 360 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 360  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))
  ;; (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  ;; ;; (reach :fix-contact? nil)
  ;; (heave #f(50 -300 0) #f(50 300 0) :reset? nil :fix-contact? 2)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? 2)

  (send *ri* :start-log)
  ;; (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-hands? t)
  ;; (heave #f(-100 -900 0) #f(-100 900 0) :reset? t :fix-contact? 2 :wait? t)
  (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t)
  (change-arm 130 :rarm)
  (change-arm 130 :larm)
  ;; (heave #f(100 -100 0) #f(100 100 0) :reset? t :fix-contact? 2 :wait? t)
  (load "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")
  (add-external-force :link "body" :robot "box1" :force #f(50 0 0) :tm 0.1)
  
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  ;; ;; (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 100)) (send *robot* :link "LLEG_LINK5"))
  ;; (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  ;; (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  ;; ;; (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 100)) (send *robot* :link "LLEG_LINK5"))
  ;; (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  ;; (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)



  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  (move-hand :rarm #f(100 -200 0))
  (move-hand :rarm #f(100 -200 0))
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(200 0 0)) :only-move? t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(200 0 0)) :only-move? t)
  (setq save-angle (send *robot* :angle-vector))
  (setq lleg-coords (send *robot* :lleg :end-coords))
  (dive #f(-100 -100 -100) #f(-100 100 100) :deg-l-x 90 :deg-l-y -20 :deg-r-z -40 :reset? t :fix-contact? nil :wait? t :avoid-collision? t  :rleg-pos-shift #f(0 100 0) :lleg-pos-shift #f(400 0 0) :send? t)
  
  ;; (send *robot* :move-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 100)) (send *robot* :link "LLEG_LINK5"))
  ;; (send *robot* :move-coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(300 0 100)) (send *robot* :link "RLEG_LINK5"))


  ;; (heave #f(0 -300 0) #f(0 300 0) :reset? t :fix-contact? nil :wait? t :look-at-hands? t :rleg-pos-shift #f(100 0 0) :lleg-pos-shift #f(300 0 0))
  ;; (heave #f(100 0 0) #f(100 0 0) :reset? t :fix-contact? nil :wait? t :look-at-hands? t :rleg-pos-shift #f(100 0 0) :lleg-pos-shift #f(300 0 0))
)
(defun dive-board-no-walk()
  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (setq rarm-coords (make-coords :pos (float-vector 360 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 360  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))
  ;; (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  ;; ;; (reach :fix-contact? nil)
  ;; (heave #f(50 -300 0) #f(50 300 0) :reset? nil :fix-contact? 2)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil)
  ;; (heave #f(0 -200 0) #f(0 200 0) :reset? nil :fix-contact? 2)

  (send *ri* :start-log)
  ;; (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-hands? t)
  ;; (heave #f(-100 -900 0) #f(-100 900 0) :reset? t :fix-contact? 2 :wait? t)
  (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0))
  (change-arm 130 :rarm)
  (change-arm 130 :larm)
  (load "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")
  (add-external-force :link "body" :robot "box1" :force #f(50 0 0) :tm 0.1)
  (setq save-lleg-coords (send *robot* :lleg :end-coords))
  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :move-coords save-lleg-coords (send *robot* :lleg :end-coords))
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5) :is-hand-fix-mode t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
    (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5) :is-hand-fix-mode nil)
  
  (setq save-lleg-coords (send *robot* :lleg :end-coords))
  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :move-coords save-lleg-coords (send *robot* :lleg :end-coords))

  (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? nil :wait? t :send? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0))
  (heave #f(0 -200 0) #f(0 200 0) :reset? t :fix-contact? 2 :wait? nil :send? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0))

  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5) :is-hand-fix-mode t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(200 0 0)) :only-move? t)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5) :is-hand-fix-mode nil)
  
  (setq save-lleg-coords (send *robot* :lleg :end-coords))
  (setq save-angle (send *ri* :state :angle-vector))
  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :move-coords save-lleg-coords (send *robot* :lleg :end-coords))
  ;; (heave #f(150 -200 0) #f(150 200 0) :reset? t :fix-contact? nil :wait? nil :send? t  :rleg-pos-shift #f(300 0 0) :lleg-pos-shift #f(300 0 0))
  ;; (setq save-lleg-coords (send *robot* :lleg :end-coords))
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  ;; (send *robot* :move-coords save-lleg-coords (send *robot* :lleg :end-coords))
  )
(defun dem-bigboard-support-simulator()
  ;; (setq flag-simulator nil)

  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)

  ;; (setq rarm-coords (make-coords :pos (float-vector 340 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  ;; (setq larm-coords (make-coords :pos (float-vector 840  310 400)  :rpy (float-vector (deg2rad 0) (deg2rad  80) (deg2rad  90))))
  (setq rarm-coords (make-coords :pos (float-vector 240 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))  
  (setq larm-coords (make-coords :pos (float-vector 840  310 400)  :rpy (float-vector (deg2rad 0) (deg2rad  70) (deg2rad  90))))

  ;; (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? nil :avoid-collision? t :apply-external-force? t)
  

  )


