(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/validate-jacobian.l")

(print "load my-ik.l")
(defun my-ik
    (rarm-coords larm-coords &key rleg lleg cog (move-root-link :z) (virtual-joint-weight #f(0.5 0.5 0.5  1 1 1)))

  (if (eq rleg nil)
      (setq rleg  (send *coords* :get-rleg))
    (format t "rleg ~A" rleg))
  (if (eq lleg nil)
      (setq lleg  (send *coords* :get-lleg)))
  (if (eq cog nil)
      (setq cog  (send *coords* :get-cog)))

  ;; (setq rleg-aaa  (send (send *coords* :get-rleg) :copy-worldcoords))
  ;; (setq lleg  (send (send *coords* :get-lleg) :copy-worldcoords))
  ;; (setq cog  (send *coords* :get-cog))

  ;; (setq a b)
  ;; (setq rleg  (send *robot* :rleg :end-coords :copy-worldcoords))
        
  (send *robot* :fullbody-inverse-kinematics
        (list rleg
              lleg
              rarm-coords
              larm-coords)
          
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              (send *robot* :larm :end-coords))
        :link-list
        (list (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
              (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
              (send *robot* :link-list (send *robot* :rarm :end-coords :parent))                 
              (send *robot* :link-list (send *robot* :larm :end-coords :parent)))
        :root-link-virtual-joint-weight virtual-joint-weight
        ;; :root-link-virtual-joint-weight #f(0.5 0.5 0.5 0.0 0.0 0.0)
        
        :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0)
                                      (list (send *robot* :torso :waist-p :child-link) 0.0)
                                      (list (send *robot* :torso :waist-r :child-link) 0.0)
                                      )
        
        ;; :translation-axis (list t t t)
        ;; :rotation-axis (list t t t)
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        :target-centroid-pos (v+ (midpoint 0.5 (send rleg :worldpos) (send lleg :worldpos))
                                 cog)
        :debug-view nil
        ;; :revert-if-fail t
        ;; :cog-translation-axis move-root-link
        :cog-translation-axis move-root-link
        )
  
  (send rarm-coords :draw-on :flush nil :size 300)
  (send larm-coords :draw-on :flush t   :size 300)
  )

(defun setup-jaxon_red-ik-opt
    (&key (only-kinematics? nil))
  ;; (setq *robot* (instance hrp2jsknts-robot :init))
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
    ;; create support polygon
    ;; (create-support-polygon *robot* (send *robot* :hand :rarm :link "RARM_LINK7"))
    ;; (create-support-polygon *robot* (send *robot* :hand :larm :link "LARM_LINK7"))
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


  ;; (send (send *robot* :joint "RARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT5") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT5") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :max-angle 50)

  ;; (send (send *robot* :joint "RARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "RARM_JOINT7") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT7") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT7") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT7") :max-angle 50) 
  
  )


;; (defun jaxon_red-sqp-optimization-instant
(defun ik-opt
    (rarm-coords larm-coords 
    &key
     (optimize-torque? t)
     (avoid-collision? t)
     (apply-external-force? nil)
     (fix-contact? nil)
     (set-kin-scale? nil)
     (only-kinematics? nil)
     (pre-process-func)
     (reset? t)
     ;; (rleg-pos #f( 0 -200 -10))
     ;; (lleg-pos #f( 0  200 -10))
     (rleg-pos-shift #f(0 0 0))
     (lleg-pos-shift #f(0 0 0))
     &allow-other-keys
     )
  (setq rleg-pos-shift (v+ rleg-pos-shift #f(0 0 -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift #f(0 0 -10)))
  ;; setup environment
  ;;脚を開きすぎるとIK too lergeになる
  ;; (setq *ground-rleg* (make-cube 1000 500 20))
  ;; (setq *ground-lleg* (make-cube 1000 500 20))
  ;; (send *ground-rleg* :newcoords
  ;;       (make-coords :pos (float-vector 0 -260 -10) :rpy (list 0 0 0))
  ;;       )
  ;; (send *ground-lleg* :newcoords
  ;;       (make-coords :pos (float-vector 0 260 -10) :rpy (list 0 0 0))
  ;;       )

  ;;
  ;; (setq *ground-rleg* (make-cube 1000 360 20))
  ;; (setq *ground-lleg* (make-cube 1000 360 20))
  (setq *ground-rleg* (make-cube 1000 400 20))
  (setq *ground-lleg* (make-cube 1000 400 20))
  ;; (send *ground-rleg* :newcoords
  ;;       (make-coords :pos (float-vector 0 (* -1 ground-y) -10) :rpy (list 0 0 0))
  ;;       )
  ;; (send *ground-lleg* :newcoords
  ;;       (make-coords :pos (float-vector 0 (*  1 ground-y) -10) :rpy (list 0 0 0))
  ;;       )
    (send *ground-rleg* :newcoords
        (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate rleg-pos-shift) :worldpos)  :rpy (list 0 0 0))
        )
  (send *ground-lleg* :newcoords
        (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate lleg-pos-shift) :worldpos)  :rpy (list 0 0 0))
        )

  
  ;; (send *ground-larm* :newcoords
  ;;       (make-coords :pos (float-vector 900 500 1000) :rpy (list 0 (deg2rad -45) (deg2rad 45))))
  ;; (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-larm*))
  (setq *env-list* (list *ground-rleg* *ground-lleg*))
  (send-all *env-list* :worldcoords)
  ;; (send-all *env-list* :set-color (float-vector 0.5 0.5 0.25))
  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-lleg* :set-color (float-vector 0 0 1))
  (setq *rarm-coords* (instance arrow-object :init))
  (send *rarm-coords* :newcoords
        ;; (make-coords :pos (float-vector 290 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90)))
        rarm-coords
        )

  (setq *larm-coords* (instance arrow-object :init))
  (send *larm-coords* :newcoords
        ;; (make-coords :pos (float-vector 290  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90)))
        larm-coords
        )
  (setq rleg-now (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-now (send *robot* :lleg :end-coords :copy-worldcoords))
 
  (when avoid-collision?
    ;; (setq *obst* (make-cube 160 600 500))
    ;; (send *obst* :translate (float-vector (- (elt (send rarm-coords :worldpos) 0) 100) 0 250))
    ;;big board version
    (format t "avoid collision~%")
    (setq *obst* (make-cube 100 1000 200)) 
    (send *obst* :translate (float-vector (- (/ (+ (elt (send rarm-coords :worldpos) 0) (elt (send larm-coords :worldpos) 0)) 2) 50) 0 100))
    (send *obst* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  (setup-jaxon_red-ik-opt :only-kinematics? only-kinematics? )
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
                     ;; (instance 2d-planar-contact :init :name "larm")
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         ;; (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))

  (if reset?
      (send *robot-env* :robot :reset-manip-pose))
  ;; (send *robot* :torso-waist-p :joint-angle 30)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *rarm-coords* *larm-coords* ) *env-list*  (if avoid-collision? (list *obst*))
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
                  :kin-target-coords-list ;;幾何到達目標位置姿勢リスト
                  (cond ((eq fix-contact? t)
                         (list (send *ground-rleg* :copy-worldcoords)
                               (send *ground-lleg* :copy-worldcoords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *larm-coords* :copy-worldcoords)                               
                               ;; (send *ground-larm* :copy-worldcoords)
                               )
                         )
                        ((eq fix-contact? nil)
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *larm-coords* :copy-worldcoords)
                               ;; (send *robot-env* :contact "larm" :contact-coords)
                               )
                         )
                        (t
                         (list rleg-now
                               lleg-now
                               (send *rarm-coords* :copy-worldcoords)
                               (send *larm-coords* :copy-worldcoords)
                               ;; (send *robot-env* :contact "larm" :contact-coords)
                               )
                         ))
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        ;; (send *robot-env* :robot :get :rarm-contact-coords)
                        ;; (send *robot-env* :robot :get :larm-contact-coords)
                        (send *robot-env* :robot  :rarm :end-coords)
                        (send *robot-env* :robot  :larm :end-coords)          
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond ((eq fix-contact? t)
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   ;; (send *ground-larm* :copy-worldcoords)
                                   )
                             )
                            ((eq fix-contact? nil)
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   ;; (send *robot-env* :contact "larm" :contact-coords)
                                   )
                             )
                            (t
                             (list
                              rleg-now
                              lleg-now
                              )
                         )))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            ;; (send *robot-env* :robot :get :larm-contact-coords)
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
                       ;; (instance default-contact-constraint
                       ;;           :init
                       ;;           :mu-trans 0.2
                       ;;           :mu-rot 0.01
                       ;;           :max-fz 200
                       ;;           :contact-face
                       ;;                  ;                                 (send *robot-env* :robot :get :larm_link6-support-polygon)
                       ;;           (send *robot-env* :robot :get :larm_link7-support-polygon)                                 
                       ;;           )
                       ))
                  :kin-scale-list
                  (if set-kin-scale?
                      (list 1.0
                            10.0
                            (list 1 2 3 4 5 6)
                            (list 1 2 3 4 5 6)
                            ))
                  :collision-pair-list
                  ;; (list
                  ;;  (list
                  ;;   (send *robot-env* :robot :link "RLEG_LINK2")
                  ;;   (send *robot-env* :robot :link "LLEG_LINK2"))
                   ;; (list
                   ;;  (send *robot-env* :robot :link "RLEG_LINK4")
                   ;;  (send *robot-env* :robot :link "LLEG_LINK4"))
                   ;; )
                  (if avoid-collision?
                      (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links)))
                  )
                  
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin 0.2
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        ;;多分絶対座標で与える
                        ;; (list (float-vector 0 -80 0 0 0 0))
                        ;; (list (float-vector 0 80 0 0 0 0))
                        ;; 力の正負が微妙かも
                        (list (float-vector 0 0 -80 0 0 0)
                              (float-vector 0 0 -80 0 0 0))
                        ;;pivot
                        ;; (list (float-vector 0 -50 0 0 0 0))
                        ;; ;;support
                        ;; ;; (list (float-vector 0 0 -50 0 0 0))
                        ;; (list (float-vector -40 0 0 0 0 0))
                        ))
                  :external-coords-list  ;;外レンチ作用点リスト
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        (list (send *robot-env* :robot :get :rarm-contact-coords)
                              (send *robot-env* :robot :get :larm-contact-coords))
                        ;;pivot
                        ;; (list (send *robot-env* :robot :get :rarm-contact-coords))
                        ;;support
                        ;; (list (send *robot-env* :robot :link "LARM_LINK5"))
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
  (send *sqp-opt* :optimize :loop-num 70)
  )
(warn "(jaxon_red-sqp-optimization-instant)~%")

(defun setup-jaxon_red-ik-opt-with-larmcontact
    (&key (only-kinematics? nil))
  ;; (setq *robot* (instance hrp2jsknts-robot :init))
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
                        :parent (send *robot* :link "LARM_LINK3")
                        :coords ;; (send
                        ;; (send (send (send (send *robot* :link "LARM_LINK3") :copy-worldcoords) :rotate (deg2rad 90) :y) :rotate (deg2rad 30) :x)
                        (send (send (send (send (send *robot* :link "LARM_LINK4") :copy-worldcoords) :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :z) :rotate (deg2rad -50) :x)
                        ;; (send (send (send (send *robot* :link "LARM_LINK5") :copy-worldcoords) :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :z)
                                      ;; :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345))
                                      ;; )
                        ))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (unless only-kinematics?
    ;; create support polygon
    ;; (create-support-polygon *robot* (send *robot* :hand :rarm :link "RARM_LINK7"))
    ;; (create-support-polygon *robot* (send *robot* :hand :larm :link "LARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "RARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "LARM_LINK4"))
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


  ;; (send (send *robot* :joint "RARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT5") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT5") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :max-angle 50)

  ;; (send (send *robot* :joint "RARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "RARM_JOINT7") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT7") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT7") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT7") :max-angle 50) 
  
  )
;;r -50
;; (dive #f(-300 400 -200) #f(-300 -200 100) :deg-l-x 90 :deg-l-y -40 :deg-r-y 40 :reset? t :fix-contact? nil :wait? t :avoid-collision? t  :rleg-pos-shift #f(-300 0 0) :lleg-pos-shift #f(0 0 0) :send? nil)
(defun ik-opt-with-larmcontact
    (rarm-coords
     larm-coords
     &key
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
     &allow-other-keys
     )
  (setq rleg-pos-shift (v+ rleg-pos-shift #f(0 0 -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift #f(0 0 -10)))  
  (setq *ground-rleg* (make-cube 600 400 20))
  (setq *ground-lleg* (make-cube 1000 400 20))
  (setq *ground-larm* (make-cube 800 600 20))
  (send *ground-rleg* :newcoords
        (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate rleg-pos-shift) :worldpos)  :rpy (list (deg2rad -50) 0 0))
        )
  (send *ground-lleg* :newcoords
        (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate lleg-pos-shift) :worldpos)  :rpy (list (deg2rad -50) 0 0))
        )
  (send *ground-larm* :newcoords
        larm-coords)
  
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-larm*))
  (send-all *env-list* :worldcoords)
  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-lleg* :set-color (float-vector 0 0 1))
  (send *ground-larm* :set-color (float-vector 0 1 0))
  (setq *rarm-coords* (instance arrow-object :init))
  (send *rarm-coords* :newcoords
        rarm-coords
        )

  (setq rleg-now (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-now (send *robot* :lleg :end-coords :copy-worldcoords))
 
  (when avoid-collision?
    ;;big board version
    (format t "avoid collision~%")
    (setq *obst* (make-cube 100 10 1000))
    (send *obst* :translate  (v+ (send (send larm-coords :copy-worldcoords) :worldpos) #f(0 300 0)))
    (send *obst* :rotate (deg2rad -30) :y)
    (send *obst* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  (setup-jaxon_red-ik-opt-with-larmcontact :only-kinematics? only-kinematics? )
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
                     (instance 2d-planar-contact :init :name "larm")
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))

  (if reset?
      (send *robot-env* :robot :reset-manip-pose))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *rarm-coords* *ground-larm*) *env-list*  (if avoid-collision? (list *obst*))
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
                               (send *rarm-coords* :copy-worldcoords)
                               (send *ground-larm* :copy-worldcoords)
                               )
                         )
                        ((eq fix-contact? nil)
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               )
                         )
                        (t
                         (list rleg-now
                               lleg-now
                               (send *rarm-coords* :copy-worldcoords)
                               (send *robot-env* :contact "larm" :contact-coords);;?????
                               )
                         ))
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        ;; (send *robot-env* :robot :get :rarm-contact-coords)
                        (send *robot-env* :robot  :rarm :end-coords)
                        (send *robot-env* :robot :get :larm-contact-coords)
                        ;; (send *robot-env* :robot  :larm :end-coords)          
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond ((eq fix-contact? t)
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   (send *ground-larm* :copy-worldcoords)
                                   )
                             )
                            ((eq fix-contact? nil)
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   (send *robot-env* :contact "larm" :contact-coords)
                                   )
                             )
                            (t
                             (list
                              rleg-now
                              lleg-now
                              (send *robot-env* :contact "larm" :contact-coords);;?????                              
                              )
                         )))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            (send *robot-env* :robot :get :larm-contact-coords)
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
                                        ;                                 (send *robot-env* :robot :get :larm_link6-support-polygon)
                                 (send *robot-env* :robot :get :larm_link4-support-polygon)
                                 )
                       ))
                  :kin-scale-list
                  (if set-kin-scale?
                      (list 1.0
                            10.0
                            (list 1 2 3 4 5 6)
                            (list 1 2 3 4 5 6)
                            ))
                  :collision-pair-list

                  (if avoid-collision?
                      (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :arms :links)))
                  )
                  
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin 0.2
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        ;;多分絶対座標で与える
                        ;; (list (float-vector 0 -80 0 0 0 0))
                        ;; (list (float-vector 0 80 0 0 0 0))
                        ;; 力の正負が微妙かも
                        (list (float-vector 0 0 -80 0 0 0)
                              ;; (float-vector 0 0 -80 0 0 0)
                              (float-vector 0 0 -80 0 0 0))
                        ;;pivot
                        ;; (list (float-vector 0 -50 0 0 0 0))
                        ;; ;;support
                        ;; ;; (list (float-vector 0 0 -50 0 0 0))
                        ;; (list (float-vector -40 0 0 0 0 0))
                        ))
                  :external-coords-list  ;;外レンチ作用点リスト
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        (list (send *robot-env* :robot :get :rarm-contact-coords)
                              ;; (send *robot-env* :robot :get :larm-contact-coords)
                              (send *robot-env* :robot  :link "LARM_LINK4"))
                        ;;pivot
                        ;; (list (send *robot-env* :robot :get :rarm-contact-coords))
                        ;;support
                        ;; (list (send *robot-env* :robot :link "LARM_LINK5"))
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
  (send *sqp-opt* :optimize :loop-num 70)
  )

;;progn (load "demo.l") (prepare-simulator)  (send *robot* :angle-vector #f(-5.80758 -6.03765 -75.0039 57.4591 -13.1998 9.56411 8.66658 8.42425 -76.3062 57.8698 -12.498 -10.2217 2.00262 3.11233 -2.03515 2.71054 -7.51743 6.97288 28.0959 -132.366 -31.2662 -94.6359 -34.0954 -10.6936 30.1544 -8.20333 23.1193 125.023 26.9899 -91.5323 29.6232 5.47784 26.1095)) (start-coords) 
;; (dive-larm-only #f(-300 -200 100) :deg-l-x 90 :deg-l-y -40  :reset? t :fix-contact? nil :wait? t :avoid-collision? t  :rleg-pos-shift #f(-300 0 0) :lleg-pos-shift #f(0 0 0) :send? nil)
(defun ik-opt-with-larmcontact-only
    (
     larm-coords
     &key
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
     &allow-other-keys
     )
  (setq rleg-pos-shift (v+ rleg-pos-shift #f(0 0 -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift #f(0 0 -10)))  
  (setq *ground-rleg* (make-cube 800 400 20))
  (setq *ground-lleg* (make-cube 800 400 20))
  (setq *ground-larm* (make-cube 800 600 20))
  (send *ground-rleg* :newcoords
        (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate rleg-pos-shift) :worldpos)  :rpy (list (deg2rad -50) 0 0))
        )
  (send *ground-lleg* :newcoords
        (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate lleg-pos-shift) :worldpos)  :rpy (list (deg2rad -50) 0 0))
        )
  (send *ground-larm* :newcoords
        larm-coords)
  
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-larm*))
  (send-all *env-list* :worldcoords)
  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-lleg* :set-color (float-vector 0 0 1))
  (send *ground-larm* :set-color (float-vector 0 1 0))

  (setq rleg-now (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-now (send *robot* :lleg :end-coords :copy-worldcoords))
 
  (when avoid-collision?
    ;;big board version
    (format t "avoid collision~%")
    ;; (setq *obst* (make-cube 100 10 1000))
    (setq *obst* (make-cube 20 600 1000))
    (send *obst* :translate  (v+ (send (send larm-coords :copy-worldcoords) :worldpos) #f(150 0 0)))
    (send *obst* :rotate (deg2rad -20) :y)
    (send *obst* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  (setup-jaxon_red-ik-opt-with-larmcontact :only-kinematics? only-kinematics? )
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
                     (instance 2d-planar-contact :init :name "larm")
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))

  (if reset?
      (send *robot-env* :robot :reset-manip-pose))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot*  *ground-larm*) *env-list*  (if avoid-collision? (list *obst*))
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
                               (send *ground-larm* :copy-worldcoords)
                               )
                         )
                        ((eq fix-contact? nil)
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               )
                         )
                        (t
                         (list rleg-now
                               lleg-now
                               (send *robot-env* :contact "larm" :contact-coords);;?????
                               )
                         ))
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        ;; (send *robot-env* :robot :get :rarm-contact-coords)
                        (send *robot-env* :robot :get :larm-contact-coords)
                        ;; (send *robot-env* :robot  :larm :end-coords)          
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond ((eq fix-contact? t)
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   (send *ground-larm* :copy-worldcoords)
                                   )
                             )
                            ((eq fix-contact? nil)
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   (send *robot-env* :contact "larm" :contact-coords)
                                   )
                             )
                            (t
                             (list
                              rleg-now
                              lleg-now
                              (send *robot-env* :contact "larm" :contact-coords);;?????                              
                              )
                         )))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            (send *robot-env* :robot :get :larm-contact-coords)
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
                                        ;                                 (send *robot-env* :robot :get :larm_link6-support-polygon)
                                 (send *robot-env* :robot :get :larm_link4-support-polygon)
                                 )
                       ))
                  :kin-scale-list
                  (if set-kin-scale?
                      (list 1.0
                            10.0
                            (list 1 2 3 4 5 6)
                            (list 1 2 3 4 5 6)
                            ))
                  :collision-pair-list

                  (if avoid-collision?
                      (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :arms :links)) )
                  )
                  
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin 0.1
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        ;;多分絶対座標で与える
                        ;; (list (float-vector 0 -80 0 0 0 0))
                        ;; (list (float-vector 0 80 0 0 0 0))
                        ;; 力の正負が微妙かも
                        (list 
                              ;; (float-vector 0 0 -80 0 0 0)
                              (float-vector -80 0 -80 0 0 0))
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
                              (send *robot-env* :robot  :link "LARM_LINK4"))
                        ;;pivot
                        ;; (list (send *robot-env* :robot :get :rarm-contact-coords))
                        ;;support
                        ;; (list (send *robot-env* :robot :link "LARM_LINK5"))
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
  (send *sqp-opt* :optimize :loop-num 100)
  )

(defun move-on-foot(init-rcoords init-lcoords next-rcoords  next-lcoords &key (which? :lleg))
  ;; (setq ldif (send (send init-lcoords :copy-worldcoords) :transformation init-lcoords))
  ;; (setq rdif (send (send init-rcoords :copy-worldcoords) :transformation next-rcoords))
  (format t "init-lcoords = ~A~%" init-lcoords)
  (format t "next-rcoords = ~A~%" next-rcoords)
  (format t "next-lcoords = ~A~%" next-lcoords)  
  (if (eq which? :lleg)
      (send *ri* :set-foot-steps
        ;; (list
        ;;  (make-coords :coords init-rcoords :name :rleg)
        ;;  (make-coords :coords (send (send init-lcoords :copy-worldcoords) :translate (send ldif :worldpos) :rpy (car (send ldif :rpy-angle))) :name :lleg)
        ;;  (make-coords :coords (send (send init-rcoords :copy-worldcoords) :translate (send rdif :worldpos) :rpy (car (send rdif :rpy-angle))) :name :rleg)
        ;;  ;; (make-coords :coords (send (send init-rcoords :copy-worldcoords) :transformation next-rcoords) :name :rleg)
        ;;  ;; (make-coords :coords (send (send init-lcoords :copy-worldcoords) :transformation next-lcoords) :name :lleg)
        ;;  )
        (list
         (make-coords :coords init-rcoords :name :rleg)
         (make-coords :coords  next-lcoords :name :lleg)
         (make-coords :coords  next-rcoords :name :rleg)
         
         ))
    (send *ri* :set-foot-steps
        ;; (list
        ;;  (make-coords :coords init-rcoords :name :rleg)
        ;;  (make-coords :coords (send (send init-lcoords :copy-worldcoords) :translate (send ldif :worldpos) :rpy (car (send ldif :rpy-angle))) :name :lleg)
        ;;  (make-coords :coords (send (send init-rcoords :copy-worldcoords) :translate (send rdif :worldpos) :rpy (car (send rdif :rpy-angle))) :name :rleg)
        ;;  ;; (make-coords :coords (send (send init-rcoords :copy-worldcoords) :transformation next-rcoords) :name :rleg)
        ;;  ;; (make-coords :coords (send (send init-lcoords :copy-worldcoords) :transformation next-lcoords) :name :lleg)
        ;;  )
        (list
         (make-coords :coords init-lcoords :name :lleg)
         (make-coords :coords  next-rcoords :name :rleg)
         (make-coords :coords  next-lcoords :name :lleg)
         
         ))
    )
  ;; (format t "rleg init ~A~%" init-rcoords)
  ;; (format t "lleg init ~A~%" init-lcoords)
  ;; (format t "rleg ~A~%" (make-coords :coords (send init-rcoords :transformation next-rcoords) :name :rleg))
  ;; (format t "lleg ~A~%" (make-coords :coords (send init-lcoords :transformation next-lcoords) :name :lleg))
  )

(defun ik-walk
    (rarm-coords larm-coords
                 &key rleg lleg cog (move-root-link :z) (virtual-joint-weight #f(0.5 0.5 0.5  1 1 1))
                 (optimize-torque? t)
                 (pre-process-func)
                 (post-process-func)
                 (stop 50)
                 )
  (if (eq rleg nil)
      (setq rleg  (send *coords* :get-rleg))
    (format t "rleg ~A" rleg))
  (if (eq lleg nil)
      (setq lleg  (send *coords* :get-lleg)))
  (if (eq cog nil)
      (setq cog  (send *coords* :get-cog)))

  (send *robot* :torso-waist-p :joint-angle 30)
  (setq *target-centroid-pos* (float-vector 300 0 0))  
    ;;ground rleg
  (setq *ground* (make-cube 3000 2000 5))  
  (send *ground* :newcoords
        (make-coords :pos (float-vector 0 -1200 0))); -923.285)));(- (- 0 (elt (send *robot* :body_lk :worldpos ) 2)) 2.5))))
                                        ;        (send (send *pull_target* :copy-worldcoords) :translate #f(0 -200 0))) ;;thickness of door
        
  (send *ground* :set-color (float-vector 1.0 0 0))

  (setq *floor1* (elt (send *ground* :faces) 3)) ;;elt 3 :normal-move-target-axis :z)  が見た感じ最適？ 
  (print (send *ground* :faces))
  (send *floor1* :put :margin 20) ;; [mm]
  (send *floor1* :put :normal-move-target-axis :z)  
  (send *floor1* :name :face-rleg)

  ;;ground2 lleg
  (setq *ground2* (make-cube 3000 2000 5))  
  (send *ground2* :newcoords
        (make-coords :pos (float-vector 0 1200 0)));;-923.285))); (- (- 0 (elt (send *robot* :body_lk :worldpos ) 2)) 2.5))))
  (send *ground2* :set-color (float-vector 0 0 1.0))  
  (setq *floor2* (elt (send *ground2* :faces) 3)) ;;elt 3 :normal-move-target-axis :z)  が見た感じ最適？ 
  (print (send *ground2* :faces))
  (send *floor2* :put :margin 20) ;; [mm]
  (send *floor2* :put :normal-move-target-axis :z)  
  (send *floor2* :name :face-lleg)

  (send *robot*
        :inverse-kinematics-optmotiongen

        (list
         ;; rleg
         ;; lleg
         (send (send *robot* :rleg :end-coords) :copy-worldcoords)
         (send (send *robot* :lleg :end-coords) :copy-worldcoords)
         ;; *floor1*
         ;; *floor2*
         ;; (send *coords* :get-rleg)
         ;; (send *coords* :get-lleg)         

              rarm-coords
              larm-coords
              ;; (make-coords :pos *target-centroid-pos*)              
              )
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              (send *robot* :larm :end-coords)
              ;; :cog

              )
        :link-list
        (list (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
              (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
              (send *robot* :link-list (send *robot* :rarm :end-coords :parent))                 
              (send *robot* :link-list (send *robot* :larm :end-coords :parent)))
        
        :contact-target-coords
        (list ;; (send (send *robot* :rleg :end-coords) :copy-worldcoords)
         ;; (send (send *robot* :lleg :end-coords) :copy-worldcoords)
         *floor1*
         *floor2*
              )
        :contact-move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              )
        ;; (list (send *coords* :get-rleg )
        ;;       (send *coords* :get-lleg )
        ;;       )
                

        :contact-constraint
        ;; (list
         ;; (instance default-contact-constraint
         ;;           :init-from-support-polygon
         ;;           :support-polygon (send  (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
         ;;           :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
         ;;           )
         ;; (instance default-contact-constraint
         ;;           :init-from-support-polygon
         ;;           :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
         ;;           :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
         ;;           )

         ;; )

        (list
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send  (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                   )
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                   )

         )
        ;; (list
        ;;  (instance default-contact-constraint
        ;;            :init-from-support-polygon
        ;;            :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
        ;;            :support-polygon-coords (send (send *coords* :get-rleg) :worldcoords)
        ;;            )
        ;;  (instance default-contact-constraint
        ;;            :init-from-support-polygon
        ;;            :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
        ;;            :support-polygon-coords (send (send *coords* :get-lleg) :worldcoords)
        ;;            )

        ;;  )
        
        ;; :translation-axis (list t t)
        ;; :rotation-axis (list t t)
        :translation-axis (list :xy :xy t t)
        :rotation-axis (list t t t t)
        ;; :debug-view t
        :root-virtual-mode :6dof
        :collision-avoidance-link-pair
        (list
         (list 
         (elt (send *robot* :lleg :links) 5)
         (elt (send *robot* :rleg :links) 5)
         )
         )
        :collision-distance-margin 100
        :optimize-torque? optimize-torque?
        :stop stop
        :min-loop 30
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        ;;cog関係は無視されてそう
        ;; :target-centroid-pos (v+ (midpoint 0.5 (send rleg :worldpos) (send lleg :worldpos))
        ;;                          cog)
        :target-centroid-pos (v+ (midpoint 0.5 (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :worldpos)
                                           (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :worldpos))
                                 #f(1000 0 0))

        :cog-translation-axis move-root-link     
        )
  
  )

(defun ik-st
    (rarm-coords larm-coords
                 &key rleg lleg cog (move-root-link :z) (virtual-joint-weight #f(0.5 0.5 0.5  1 1 1))
                 (optimize-torque? t)
                 (pre-process-func)
                 (post-process-func)
                 (stop 30)
                 )

  (if (eq rleg nil)
      (setq rleg  (send *coords* :get-rleg))
    (format t "rleg ~A" rleg))
  (if (eq lleg nil)
      (setq lleg  (send *coords* :get-lleg)))
  (if (eq cog nil)
      (setq cog  (send *coords* :get-cog)))

  ;; (create-support-polygon *robot* (send (send *coords* :get-rleg) :parent))
  ;; (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))  

  ;;ground rleg
  (setq *ground* (make-cube 3000 2000 5))  
  (send *ground* :newcoords
        (make-coords :pos (float-vector 0 0 0))); -923.285)));(- (- 0 (elt (send *robot* :body_lk :worldpos ) 2)) 2.5))))
                                        ;        (send (send *pull_target* :copy-worldcoords) :translate #f(0 -200 0))) ;;thickness of door
        
  (send *ground* :set-color (float-vector 1.0 0 0))

  (setq *floor1* (elt (send *ground* :faces) 1)) ;;elt 3 :normal-move-target-axis :z)  が見た感じ最適？ 
  (print (send *ground* :faces))
  (send *floor1* :put :margin 20) ;; [mm]
  (send *floor1* :put :normal-move-target-axis :z)  
  (send *floor1* :name :face-rleg)

  ;;ground2 lleg
  (setq *ground2* (make-cube 3000 2000 5))  
  (send *ground2* :newcoords
        (make-coords :pos (float-vector 0 0 0)));;-923.285))); (- (- 0 (elt (send *robot* :body_lk :worldpos ) 2)) 2.5))))
  (send *ground2* :set-color (float-vector 0 0 1.0))  
  (setq *floor2* (elt (send *ground2* :faces) 1)) ;;elt 3 :normal-move-target-axis :z)  が見た感じ最適？ 
  (print (send *ground2* :faces))
  (send *floor2* :put :margin 20) ;; [mm]
  (send *floor2* :put :normal-move-target-axis :z)  
  (send *floor2* :name :face-lleg)


  (send *robot*
        :inverse-kinematics-statics-optmotiongen
        ;;:inverse-kinematics-trajectory-optmotiongen
        (list
         rleg
         lleg
         ;; *floor1*
         ;; *floor2*
              rarm-coords
              larm-coords
              )
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              (send *robot* :larm :end-coords)
              )
        :contact-target-coords
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              )
        ;; (list *floor1* *floor2*)
        :contact-move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              )
        :contact-constraint
        (list
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                   )
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                   )
         )

        ;; :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0)
        ;;                               (list (send *robot* :torso :waist-p :child-link) 0.0)
        ;;                               (list (send *robot* :torso :waist-r :child-link) 0.0)
        ;;                               )
        ;; :cog-translation-axis move-root-link
        
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        ;; :translation-axis (list t t )
        ;; :rotation-axis (list t t )
        
        :debug-view t
;;        :root-virtual-mode :6dof ;;root fix version
        :root-virtual-mode :6dof ;:planar  ;;virtual joint version
        :optimize-torque? optimize-torque?
        :stop stop
        :min-loop 30
;;??        :root-virtual-joint-invariant? t        
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
