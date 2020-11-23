(load "demo.l")
(load "make_plane_for_opt.l")
(defun setup-jaxon_red-ik-change-arms
    (&key (only-kinematics? nil))
  (setq *robot* (instance jaxon_red-robot :init))
  ;; assoc contact coords
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "RLEG_LINK5")
                        :coords (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "LLEG_LINK5")
                        :coords (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "RARM_LINK7")
                        :coords (send (send (send *robot* :link "RARM_LINK7") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 -0.0233 -0.2345)))
                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "LARM_LINK7")
                        :coords (send (send (send *robot* :link "LARM_LINK7") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345)))
                        ))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (unless only-kinematics?
    ;; (create-support-polygon *robot* (send *robot* :link "RARM_LINK7"))
    ;; (create-support-polygon *robot* (send *robot* :link "LARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "RARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "LARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "RLEG_LINK5"))
    (create-support-polygon *robot* (send *robot* :link "LLEG_LINK5"))
    )
  ;; disable min-max table
  (dolist (jnt (send *robot* :joint-list))
    (send jnt :set-val 'joint-min-max-table nil)
    (send jnt :set-val 'joint-min-max-target nil)
    )
  ;;set narrow limit of min/max angle instead of min-max table
  ;; (send (send *robot* :joint "RLEG_JOINT0") :min-angle -35)
  ;; (send (send *robot* :joint "LLEG_JOINT0") :max-angle 35)
  ;; (send (send *robot* :joint "RLEG_JOINT2") :min-angle -70)
  ;; (send (send *robot* :joint "LLEG_JOINT2") :min-angle -70)
  (send (send *robot* :joint "CHEST_JOINT1") :min-angle 0)
  (send (send *robot* :joint "CHEST_JOINT1") :max-angle 30)
  
  (send (send *robot* :joint "RLEG_JOINT0") :max-angle 20)
  (send (send *robot* :joint "LLEG_JOINT0") :min-angle -20)
  (send (send *robot* :joint "RLEG_JOINT3") :max-angle 60)
  (send (send *robot* :joint "LLEG_JOINT3") :max-angle 60)
  
  (send (send *robot* :joint "RARM_JOINT0") :min-angle 0)
  (send (send *robot* :joint "RARM_JOINT0") :max-angle 40)
  (send (send *robot* :joint "LARM_JOINT0") :min-angle -40)
  (send (send *robot* :joint "LARM_JOINT0") :max-angle 0)
  ;;肩の特異点問題dem-step-simulator
  ;; (send (send *robot* :joint "RARM_JOINT1") :min-angle -100)
  ;; (send (send *robot* :joint "RARM_JOINT1") :max-angle 80)
  ;; (send (send *robot* :joint "LARM_JOINT1") :min-angle -100)
  ;; (send (send *robot* :joint "LARM_JOINT1") :max-angle 80)

  (send (send *robot* :joint "RARM_JOINT1") :min-angle -140)
  (send (send *robot* :joint "RARM_JOINT1") :max-angle 140)
  (send (send *robot* :joint "LARM_JOINT1") :min-angle -140)
  (send (send *robot* :joint "LARM_JOINT1") :max-angle 140)

  
  )


(defun ik-opt-change-arm
    (arm-coords
     &key
     (arm "larm")
     (optimize-torque? t)
     (avoid-collision? t)
     (apply-external-force? nil)
     (fix-contact? nil)
     (set-kin-scale? nil)
     (only-kinematics? nil)
     (pre-process-func)
     (reset? t)
     (rleg-pos-shift #f(0 0 0))
     (lleg-pos-shift #f(0 0 0))
     (face-arm (make-cube 800 600 20))
     (face-arm-com com)
     &allow-other-keys
     )

  (if (eq arm "larm")
      (setq oparm "rarm")
    (setq oparm "larm")
    )
  (setq rleg-pos-shift (v+ rleg-pos-shift #f(0 0 -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift #f(0 0 -10)))  
  (setq *ground-rleg* (make-cube 800 400 20))
  (setq *ground-lleg* (make-cube 800 400 20))
  (setq *ground-arm* face-arm)
  (send *ground-rleg* :newcoords
        (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate rleg-pos-shift) :worldpos)  :rpy (list (deg2rad 0) 0 0)))
  (send *ground-lleg* :newcoords
        (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate lleg-pos-shift) :worldpos)  :rpy (list (deg2rad 0) 0 0)))
  ;; (send *ground-arm* :newcoords
  ;;       (make-coords :pos (send (send face-arm-com :copy-worldcoords) :worldpos) :rpy (list (deg2rad 0) 0 0))
  ;;       )
  ;; (send *ground-arm* face-arm)
  
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-arm*))
  (send-all *env-list* :worldcoords)
  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-lleg* :set-color (float-vector 0 0 1))
  (send *ground-arm* :set-color (float-vector 0 1 0))
  (setq *arm-coords* (instance arrow-object :init))
  (send *arm-coords* :newcoords arm-coords  )

  (setq rleg-now (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-now (send *robot* :lleg :end-coords :copy-worldcoords))
 
  (when avoid-collision?
    ;;big board version
    (format t "avoid collision~%")
    (setq tilt-y -40)
    (setq *obst* (make-cube 100 10 1000))
    ;; (setq *obst* (make-cube 20 600 1000))
    ;; (send *obst* :translate  (v+ (send (send larm-coords :copy-worldcoords) :worldpos) #f(150 0 0)))
    ;; (send *obst* :rotate (deg2rad tilt-y) :y)
    ;; (send *obst* :set-color (float-vector 1 0 0) 0.5)

    ;; (setq *obst-side* (make-cube 300 20 1000))
    ;; (send *obst-side* :translate  (v+ (send (send larm-coords :copy-worldcoords) :worldpos) #f(150 300 0)))
    ;; (send *obst-side* :rotate (deg2rad tilt-y) :y)
    ;; (send *obst-side* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  ;; (setup-jaxon_red-ik-opt-with-larmcontact :only-kinematics? only-kinematics? ) 
  (setup-jaxon_red-ik-change-arms :only-kinematics? only-kinematics? ) ;;contact coords larm を少しずらした
  ;;fix-contact nil 足を自由な位置へ
  ;;t contact-coordsの位置へ
  ;;2 その場で
  (cond (fix-contact?
         (setq *robot-env*
               (instance robot-environment :init :robot *robot*))
         )
        (t
         (setq *contact-list*
               (list (instance 2d-planar-contact :init :name "rleg")
                     (instance 2d-planar-contact :init :name "lleg")
                     (instance 2d-planar-contact :init :name  (format nil "~A" arm))
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         (send *robot-env* :contact (format nil "~A" arm) :set-from-face :face (elt (send *ground-arm* :faces) 1))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))

  (if reset?
      (send *robot-env* :robot :reset-manip-pose)
    )
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  
  (objects (append (list *robot* *arm-coords* *ground-arm*) *env-list*  (if avoid-collision? (list *obst* ;; *obst-side*
                                                                                                   ))
                   ))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if (and (not only-kinematics?) optimize-torque?)
                      (send *robot-env* :drive-joint-list) nil)
                  :only-kinematics? only-kinematics?
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send-all (send *robot-env* :robot :joint-list) :joint-angle)
                  :kin-target-coords-list ;;幾何到達目標位置姿勢リスト t以外なら左手は平面
                  (cond ((eq fix-contact? t)
                         (list (send *ground-rleg* :copy-worldcoords)
                               (send *ground-lleg* :copy-worldcoords)
                               (send *arm-coords* :copy-worldcoords)
                               (send *ground-arm* :copy-worldcoords)
                               )
                         )
                        ((eq fix-contact? nil)
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *arm-coords* :copy-worldcoords)
                               (send *robot-env* :contact (format nil "~A" arm) :contact-coords)
                               )
                         )
                        (t
                         (list rleg-now
                               lleg-now
                               (send *arm-coords* :copy-worldcoords)
                               (send *robot-env* :contact (format nil "~A" arm) :contact-coords)
                               )
                         ))
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        (send *robot-env* :robot  (read-from-string (format nil ":~A" oparm)) :end-coords)
                        (send *robot-env* :robot :get (read-from-string (format nil ":~A-contact-coords" arm)))
                        ;; (send *robot-env* :robot :get :larm-elbow-contact-coords)

                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond ((eq fix-contact? t)
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   (send *ground-arm* :copy-worldcoords)
                                   ;; (send *ground-larm* :copy-worldcoords);;リンクの長さ分ずらしたほうが良さそう？
                                   )
                             )
                            ((eq fix-contact? nil)
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   ;; (send *robot-env* :contact (read-from-string (format nil "~A" oparm)) :contact-coords)
                                   (send *robot-env* :contact (format nil "~A" arm) :contact-coords)
                                   )
                             )
                            (t
                             (list
                              rleg-now
                              lleg-now
                              ;; (send *robot-env* :contact (read-from-string (format nil "~A" oparm)) :contact-coords);;?????
                              (send *robot-env* :contact (format nil "~A" arm) :contact-coords)
                              )
                         )))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            (send *robot-env* :robot :get (read-from-string (format nil ":~A-contact-coords" arm)))
                            ))
                  :contact-constraint-list
                  (if (not only-kinematics?)
                      (list
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :rleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :lleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance default-contact-constraint
                                 :init
                                 :mu-trans 0.2
                                 :mu-rot 0.01
                                 :max-fz 200
                                 :contact-face
                                 (send *robot-env* :robot :get (read-from-string (format nil ":~A_link7-support-polygon" arm))
                                 ))
                       ))
                  :kin-scale-list
                  (if set-kin-scale?
                      (list 1.0
                            10.0
                            (list 1 2 3 4 5 6)
                            (list 1 2 3 4 5 6)
                            (list 1 2 3 4 5 6)                            
                            ))
                  :collision-pair-list
                  (if avoid-collision?
                      ;; (append
                       (mapcar #'(lambda (l) (list l *obst* )) (flatten (send *robot-env* :robot :arms :links)))
                       ;; (mapcar #'(lambda (l) (list l *obst-side*)) (flatten (send *robot-env* :robot :arms :links)))
                       
                  )
                  
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin  0.1
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        (list 
                              ;; (float-vector 0 0 -80 0 0 0)
                              (float-vector -40 0 -10 0 0))
                        ;;pivot
                        ;; (list (float-vector 0 -50 0 0 0 0))
                        ;; ;;support
                        ;; ;; (list (float-vector 0 0 -50 0 0 0))
                        ;; (list (float-vector -40 0 0 0 0 0))
                        ))
                  :external-coords-list  ;;外レンチ作用点リスト
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        (list 
                              ;; (send *robot-env* :robot :get :larm-contact-coords)
                              (send *robot-env* :robot  :link "LARM_LINK7"))
                        )
                    )
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  )

(setq face-arm (make-cube 800 600 20))
;; (send face-arm :newcoords
;;       (make-coords :pos (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(100 0 0)) :worldpos)  :rpy (list (deg2rad 0) 0 0))
;;       )
(setq face-arm-com (make-coords :pos (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(100 0 0)) :worldpos)))
(setq arm-coords  (make-coords :pos (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(100 0 0)) :worldpos)  :rpy (list (deg2rad 0) 0 0)))
(setq arm "larm")

(defun change
    (&key (fix-contact? nil)
          (send? t)
          (wait? t)
          (avoid-collision? nil)
          (apply-external-force? nil)
          (rleg-pos-shift #f(0 0 -10))
          (lleg-pos-shift #f(0 0 -10))
          (look-at-object? nil)
          (arm arm)
          (face-arm face-arm)
          (face-arm-com face-arm-com)
          )
  
  (setq face-arm (make_plane))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (ik-opt-change-arm  arm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :face-arm face-arm :face-arm-com face-arm-com)

  (setq zmp-vector (send *ri*  :zmp-vector))
  (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  (if send?
      (progn 
        (if (not fix-contact?)
            (progn
              (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
              (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t)))

        ;;もどす
        ;; (send-pose movetime :force-move flag-simulator :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?  )
        ))
    )

(defun test-change-arm()
  (my-init-pose :move-foot? nil)
  (start-coords)
  (send *ri* :start-impedance :arms)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (setq rarm-coords (make-coords :pos (float-vector 360 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 360  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))

  (send *ri* :set-log-maxlength (* 500 60))
  (send *ri* :start-log)
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-object? t)
  ;; (heave #f(-110 -900 0) #f(-110 900 0) :reset? t :fix-contact? 2 :wait? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0) :look-at-object? t)
  (heave #f(-110 -1000 0) #f(-110 1000 0) :reset? t :fix-contact? 2 :wait? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0) :look-at-object? t)

    (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)
  (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 0)) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :only-move? t)

  ;; (setq face (make_plane))
  ;; (change)
)
