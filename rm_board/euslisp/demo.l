(load "./functions.l")
;; (load "./old/vision.l")

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

(defun demo ()
  (my-init-pose)
  
  (reach)
  (heave #f(100  -400 0) #f(100  400 0))
  (heave #f(100  -300 0) #f(100  300 0))

  )

(defun dem-simulator()

  (my-init-pose)
  (reach)
  (heave #f(100 0 0) #f( 100 0 0) :cog #f(100 0 700))
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)

  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  
  (heave #f(0 -50 0) #f(0 50 0))

  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700))


  (heave #f(60 -400 0) #f(60 400 0) :cog #f(100 0 800))

  (stand-up 1000  :cog #f(100 0 900))

  (stand-up 1000  :cog #f(100 0 900))

  (heave #f(60 -100 0) #f(60 100 0) :cog #f(100 0 1000))
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
  (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t)
  

  (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t)
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
  (heave #f(300 200 0) #f(400 -700 200) :deg-l-y -30 :deg-r-y 30 :reset? t :fix-contact? nil :wait? t :avoid-collision? nil  :rleg-pos #f(-600 -50 -10) :lleg-pos #f(-200 250 -10))
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
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-object? t)

  (setq pre-angle (send *robot* :angle-vector))
  
  (heave #f(-80 -800 0) #f(-80 800 0) :reset? t :fix-contact? 2 :wait? t :look-at-object? t)
  )
;;1026実機実験用
(defun dem-bigboard-pre(simulator?)
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
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-object? t)

  (setq pre-angle (send *robot* :angle-vector))
  (if simulator? nil
    (setq movetime 20000)
    )
  
  (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t :look-at-object? t)
  (if simulator? nil
    (setq movetime 16000)
    )
  )

(defun dem-bigboard-mid(simulator?)
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

  (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? nil :avoid-collision? t :apply-external-force? t)
  
  (heave #f(-80 -700 0) #f(-80 700 0) :send? nil :reset? t :fix-contact? 2 :wait? t)
  ;; (heave #f(-80 -800 0) #f(-80 800 0) :reset? t :fix-contact? 2 :wait? t)  
  (change-arm 140 :rarm :send? nil)
  (change-arm 140 :larm :send? nil)
  ;; (change-arm 160 :rarm :send? nil)
  ;; (change-arm 160 :larm :send? nil)
  (heave #f(100 -200 0) #f(100 200 0) :send? nil :reset? t :fix-contact? nil :wait? nil)

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
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-object? t)
  ;; (heave #f(-100 -900 0) #f(-100 900 0) :reset? t :fix-contact? 2 :wait? t)
  (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t)
  (change-arm 130 :rarm)
  (change-arm 130 :larm)
  ;; (heave #f(100 -100 0) #f(100 100 0) :reset? t :fix-contact? 2 :wait? t)
  (load "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")
  (add-external-force :link "body" :robot "box1" :force #f(50 0 0) :tm 0.1)
  
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)

  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  (move-hand :rarm #f(100 -200 0))
  (move-hand :rarm #f(100 -200 0))
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(200 0 0)) :only-move? t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(200 0 0)) :only-move? t)
  (setq save-angle (send *robot* :angle-vector))
  (setq lleg-coords (send *robot* :lleg :end-coords))
  (dive #f(-100 -100 -100) #f(-100 100 100) :deg-l-x 90 :deg-l-y -20 :deg-r-z -40 :reset? t :fix-contact? nil :wait? t :avoid-collision? t  :rleg-pos-shift #f(0 100 0) :lleg-pos-shift #f(400 0 0) :send? t)
  
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
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-object? t)
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

;; progn (load "demo.l" ) (prepare-simulator)
;; board-holding-pose :send? t :fix-contact? t
;; change-arm-auto :send? nil
;;progn (board-holding-pose :send? t)
;;(move-hand :larm #f(-120 0 0) :sync? nil) (move-hand :larm #f(0 -120 0) :sync? nil) (send *ri* :start-grasp :larm) (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5) :is-hand-fix-mode t)

  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn
  ;;       (send *ri* :start-log)
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             ;; (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5) :is-hand-world-fix-mode t)
  ;;             (send-pose movetime-tmp :force-move flag-simulator :pre-pose pre-angle :pre-lleg pre-lleg :look-at-object? look-at-object? :wait? nil)
  ;;             (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t :force-move t)
  ;;             ;; (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5) :is-hand-world-fix-mode nil)
  ;;             ))
  ;;       ;; (send-pose movetime :force-move flag-simulator :pre-pose pre-angle :pre-lleg pre-lleg :look-at-object? look-at-object?)
  ;;       ))
  


(defun test-change-arm()
  ;; (board-holding-pose)
  (send *ri* :start-impedance :arms)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  
  (setq coords-list (auto-set-reaching-targets))
  (setq larm-coords (elt coords-list 0))
  (setq rarm-coords (elt coords-list 1))
  ;; (send *ri* :set-log-maxlength (* 500 60))
  ;; (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-object? t)
  )
(defun ttt()
  (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0) :look-at-object? t)
  (setq flag  (check-while-working))
  (retry-flag-checker flag)
  ;; (heave #f(-110 -1000 0) #f(-110 1000 0) :reset? t :fix-contact? 2 :wait? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0) :look-at-object? t)

  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)

  (send_box_size :size-vector #f(1.0 1.5 1.0)) ;; :box-position #f(1 0 1))
  ;; (move-hand :larm #f(-140 0 0) :sync? nil)
  ;; (setq face (make_plane))
  ;; (change)
  )



(defun dem-change-arm-from-holding-pose()
  (my-init-pose :move-foot? nil)
  (send *robot* :angle-vector (send *robot* :state :angle-vector))
  (move-hand :larm #f(-140 0 0) :sync? nil)
  )
  
