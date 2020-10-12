(load "/home/utaro/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(require "models/arrow-object.l")
;; (load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/sample/sample-sqp-optimization-instant.l")

(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

(objects (list *robot*))

(defun sample-robot-reach-ik-dual-arm-with-torso
    (&key
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow-right* (instance arrow-object :init))
  (send *arrow-right* :newcoords
        ;; (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
        (make-coords :pos (float-vector 600 -300 700) :rpy (list 0 0 0)))        
  (setq *arrow-left* (instance arrow-object :init))
  (send *arrow-left* :newcoords
        ;; (make-coords :pos (float-vector 400 200 600) :rpy (list 0 0 0)))
     (make-coords :pos (float-vector 700 200 600) :rpy (list 0 0 0)))        
  (objects (list *robot* *arrow-right* *arrow-left*))

  (send *robot*
        (if irteus-ik? :inverse-kinematics :inverse-kinematics-optmotiongen)
        (list (send *arrow-right* :copy-worldcoords) (send *arrow-left* :copy-worldcoords))
        :link-list
        (list (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent))
              (send *robot* :link-list (send (send *robot* :larm :end-coords) :parent)))
        :move-target
        (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords))
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-ik-dual-arm-with-torso)~%")

(defun sample-robot-reach-ik-fullbody
    (&key
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow-right* (instance arrow-object :init))
  (send *arrow-right* :newcoords
        ;; (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0))
        (make-coords :pos (float-vector 600 -300 700) :rpy (list 0 0 0))
        )
  (setq *arrow-left* (instance arrow-object :init))
  (send *arrow-left* :newcoords
        ;; (make-coords :pos (float-vector 400 200 600) :rpy (list 0 0 0))
        (make-coords :pos (float-vector 700 200 600) :rpy (list 0 0 0))
        )
  (objects (list *robot* *arrow-right* *arrow-left*))
  (setq *target-centroid-pos* (float-vector 0 -100 0))

  (send *robot*
        (if irteus-ik? :fullbody-inverse-kinematics :inverse-kinematics-optmotiongen)
        (remove
         nil
         (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
               (send (send *robot* :lleg :end-coords) :copy-worldcoords)
               (send *arrow-right* :copy-worldcoords)
               (send *arrow-left* :copy-worldcoords)
               (if (not irteus-ik?) (make-coords :pos *target-centroid-pos*))
               ))
        :link-list
        (list (send *robot* :link-list (send (send *robot* :rleg :end-coords) :parent))
              (send *robot* :link-list (send (send *robot* :lleg :end-coords) :parent))
              (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent))
              (send *robot* :link-list (send (send *robot* :larm :end-coords) :parent))
              )
        :move-target
        (remove
         nil
         (list (send *robot* :rleg :end-coords)
               (send *robot* :lleg :end-coords)
               (send *robot* :rarm :end-coords)
               (send *robot* :larm :end-coords)
               (if (not irteus-ik?) :cog)
               ))
        :translation-axis
        (remove nil (list t t t t (if (not irteus-ik?) :z)))
        :debug-view t
        :root-virtual-mode :6dof ;; only for optmotiongen ik
        :target-centroid-pos *target-centroid-pos* ;; only for irteus ik
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-ik-fullbody)~%")

(defun sample-robot-reach-iks-face
    (&key
     (optimize-torque? t)
     (pre-process-func)
     (post-process-func)
     (stop 50)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :put :larm-contact-coords
        (make-cascoords
         :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :rotate -pi/2 :y)
         :parent (send (send *robot* :larm :end-coords) :parent)
         ))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 600 -400 800) :rpy (list 0 0 0)))
  (setq *wall* (make-cube 300 200 5))
  (send *wall* :newcoords
        (make-coords :pos (float-vector 500 200 500)
                     :rpy (list 0 (deg2rad -60) 0)))
  (send *wall* :set-color (float-vector 1.0 0.6 0.6))
  (objects (list *robot* *arrow* *wall*))

  (setq *face* (elt (send *wall* :faces) 1))
  (send *face* :put :margin 20) ;; [mm]
  (send *face* :put :normal-move-target-axis :z)
  (send *face* :name :face-larm)
  (send *robot*
        :inverse-kinematics-statics-optmotiongen
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              (send *arrow* :copy-worldcoords)
              *face*
              )
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              (send *robot* :get :larm-contact-coords)
              )
        :contact-target-coords
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              *face*
              )
        :contact-move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :get :larm-contact-coords)
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
         (instance default-contact-constraint
                   :init-from-support-polygon :support-polygon nil)
         )
        :translation-axis (list t t t t)
        :rotation-axis (list t t nil t)
        :debug-view t
        :root-virtual-mode :6dof
        :optimize-torque? optimize-torque?
        :stop stop
        :min-loop 30
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-iks-face)~%")


(defun setup-hrp2jsknts-for-sample-sqp-optimization
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
  (send (send *robot* :joint "CHEST_JOINT1") :max-angle 20)
  (send (send *robot* :joint "RARM_JOINT1") :min-angle -100)
  (send (send *robot* :joint "RARM_JOINT1") :max-angle 60)
  (send (send *robot* :joint "LARM_JOINT1") :min-angle -100)
  (send (send *robot* :joint "LARM_JOINT1") :max-angle 60)
  (send (send *robot* :joint "RARM_JOINT0") :min-angle 0)
  (send (send *robot* :joint "RARM_JOINT0") :max-angle 40)
  (send (send *robot* :joint "LARM_JOINT0") :min-angle -40)
  (send (send *robot* :joint "LARM_JOINT0") :max-angle 0)  

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


(defun sample-sqp-optimization-instant
    (&key
     (optimize-torque? t)
     (avoid-collision? t)
     (apply-external-force? nil)
     (fix-contact? nil)
     (set-kin-scale? nil)
     (only-kinematics? nil)
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup environment
  ;; (setq *ground-rleg* (make-cube 400 400 20))
  ;; (setq *ground-lleg* (make-cube 400 400 20))
  (setq *ground-rleg* (make-cube 800 400 20))
  (setq *ground-lleg* (make-cube 800 400 20))  
  ;; (setq *ground-larm* (make-cube 500 500 20))

  (send *ground-rleg* :newcoords
        (make-coords :pos (float-vector 0 -250 0) :rpy (list 0 0 0))
        )
  (send *ground-lleg* :newcoords
        (make-coords :pos (float-vector 0 250 0) :rpy (list 0 0 0))
        )
  
  ;; (send *ground-larm* :newcoords
  ;;       (make-coords :pos (float-vector 900 500 1000) :rpy (list 0 (deg2rad -45) (deg2rad 45))))
  ;; (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-larm*))
  (setq *env-list* (list *ground-rleg* *ground-lleg*))
  (send-all *env-list* :worldcoords)
  (send-all *env-list* :set-color (float-vector 0.5 0.5 0.25))
  (setq *rarm-coords* (instance arrow-object :init))
  (send *rarm-coords* :newcoords
        ;; (make-coords :pos (float-vector 1050 -500 1100) :rpy (list (deg2rad -45) -pi/2 0))
        ;; (make-coords :pos (float-vector 1050 -500 1100) :rpy (list (deg2rad -45) 0 0))
        (make-coords :pos (float-vector 290 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90)))        
        )

  (setq *larm-coords* (instance arrow-object :init))
  (send *larm-coords* :newcoords
        ;; (make-coords :pos (float-vector 1050 500 1100) :rpy (list (deg2rad -45) -pi/2 0))
        ;; (make-coords :pos (float-vector 1050 500 1100) :rpy (list (deg2rad -45) 0 0))
        (make-coords :pos (float-vector 290  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90)))
        )

  (when avoid-collision?
    (setq *obst* (make-cube 200 200 500))
    (send *obst* :translate (float-vector 500 0 250))
    (send *obst* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  (setup-hrp2jsknts-for-sample-sqp-optimization :only-kinematics? only-kinematics?)
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
         ;; (send *robot-env* :contact "rleg" :rotate pi :z :local)
         ;; (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *rarm-coords* *larm-coords* ) *env-list* (if avoid-collision? (list *obst*))))

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
                  :kin-target-coords-list
                  (cond (fix-contact?
                         (list (send *ground-rleg* :copy-worldcoords)
                               (send *ground-lleg* :copy-worldcoords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *larm-coords* :copy-worldcoords)                               
                               ;; (send *ground-larm* :copy-worldcoords)
                               )
                         )
                        (t
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *larm-coords* :copy-worldcoords)
                               ;; (send *robot-env* :contact "larm" :contact-coords)
                               )
                         ))
                  :kin-attention-coords-list
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        (send *robot-env* :robot :get :rarm-contact-coords)
                        (send *robot-env* :robot :get :larm-contact-coords)
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond (fix-contact?
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   ;; (send *ground-larm* :copy-worldcoords)
                                   )
                             )
                            (t
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   ;; (send *robot-env* :contact "larm" :contact-coords)
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
                                  :max-fz 400
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
                                  :max-fz 400
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
                  ;;  (send *robot* :link "RLEG_LINK3")
                  ;;  (send *robot* :link "LLEG_LINK3"))
                  (if avoid-collision?
                      (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links)))
                  )
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (list (float-vector 0 100 0 0 0 0)))
                  :external-coords-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        (list (send *robot-env* :robot :get :rarm-contact-coords))
                        (list (send *robot-env* :robot :get :larm-contact-coords))))
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
(warn "(sample-sqp-optimization-instant)~%")

(defun tes(send *robot*
         :inverse-kinematics-statics-optmotiongen
          (list
           *floor1*
           *floor2*
              rarm-coords
              larm-coords
                )
          
         :move-target

          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              (send *robot* :larm :end-coords)
              
                )
          ;; (list (send *robot* :rleg :end-coords)
         :contact-target-coords

          (list
           ;; (send (send *robot* :rleg :end-coords) :copy-worldcoords)
           ;; (send (send *robot* :lleg :end-coords) :copy-worldcoords)
           *floor1*
           *floor2*
           )
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
         :translation-axis

          (list t t t t);(list :x :y));;(list :x :y) (list :x :y) t)
         :rotation-axis-list

          (list t t :x :x);(list :y :z))
         :debug-view (list :additional-draw-objects (list *robot* *ground* *ground2*)) ;;*lhand_end_target* *wall*))
  ;;      :root-virtual-mode :planer
         :root-virtual-mode :6dof
         ;; :root-virtual-joint-invariant? t
         :collision-avoidance-link-pair
         (list
          (list
         (elt (send *robot* :lleg :links) 5)
         (elt (send *robot* :rleg :links) 5)
         )
          )
         :collision-distance-margin 100
         :optimize-torque? optimize-torque?
         ;; :optimize-start-end-torque? optimize-start-end-torque?
         :stop stop
         :min-loop 30
         ;; :norm-regular-scale-offset 1e-5
         ;; :adjacent-regular-scale 1e-5
         ;; :torque-regular-scale 1e-6
         :thre 5
         :rthre (deg2rad 2)
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         )
  
