(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")
;; (load "package://control_tools/log-tools.l")
;; (require "package://control_tools/play-hrpsysfile.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")

(require "../../euslisp/save-log.l")
(require "~/research/rm_debris/src/predict_from_multi_pose.l")

(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )


(objects (list *robot*))
(unless (boundp 'flag-simulator)
  (setq flag-simulator nil) ;;simulatorではｔ
  (setq movetime 10000))
;;(send *ri* :save-log "~/matsuura/log/testes" :make-directory t)

(defun prepare-simulator()
  
  (setq flag-simulator t) ;;simulatorではｔ
  (setq movetime 4000)
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st))

  )

(defclass my-coords
  :super propertied-object
  :slots (rarm-coords larm-coords rleg-coords lleg-coords cog rleg-support-polygon lleg-support-polygon)
  )

(defmethod my-coords
  (:init (rarm-val larm-val rleg-val lleg-val cg) ;;rleg-support-polygon lleg-support-polygon)
         (progn
           (send self :set-rarm rarm-val)
           (send self :set-larm larm-val)
           (send self :set-rleg rleg-val)
           (send self :set-lleg lleg-val)
           (send self :set-cog cg)
           )
         self)
  (:set-rarm (val)
             (setq rarm-coords val))
  (:set-larm (val)
             (setq larm-coords val))
  (:set-rleg (val)
             (setq rleg-coords val))
  (:set-lleg (val)
             (setq lleg-coords val))
  (:set-cog (val)
            (setq cog val))
  (:set-rleg-support-polygon (val)
                             (setq rleg-support-polygon val))
  (:set-lleg-support-polygon (val)
                             (setq lleg-support-polygon val))
 
  (:get-rarm ()
             rarm-coords)
  (:get-larm ()
             larm-coords)
  (:get-rleg ()
             rleg-coords)
  (:get-lleg ()
             lleg-coords)
  (:get-cog ()
            cog)
  (:get-rleg-polygon ()
                     rleg-support-polygon)
  (:get-lleg-polygon ()
             lleg-support-polygon)    
  )

(defclass poses
  :super propertied-object
  :slots (fin-pose-ri fin-pose-robot )
  )
(defmethod poses
  (:init (pose-robot pose-ri)
         (send self :set-fin-pose-robot pose-robot)
         (send self :set-fin-pose-ri pose-ri)
         self
         )
  (:set-fin-pose-ri (val)
                    (setq fin-pose-ri val))
  (:set-fin-pose-robot (val)
                 (setq fin-pose-robot val))  
  (:get-fin-pose-ri ()
                    fin-pose-ri )
  (:get-fin-pose-robot ()
                    fin-pose-robot )
    
  )

(setq *coords* (instance my-coords :init
                       (copy-object (send *robot* :rarm :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :larm :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :rleg :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :lleg :end-coords :copy-worldcoords))
                       #f(0 0 700)
                       ;; (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
                       ;; (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
                       ))

(setq *pose* (instance poses :init
                       (send *robot* :angle-vector)
                       (send *ri* :state :angle-vector)))
;;(float-vector 0 0 (elt (send (car (send *robot* :links)) :worldpos) 2))
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

(defun ik-walk
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

  ;; (send *robot* :torso-waist-p :joint-angle 30)
  (setq *target-centroid-pos* (float-vector 300 0 0))  
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
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
         (send (send *robot* :lleg :end-coords) :copy-worldcoords)
         ;; *floor1*
         ;; *floor2*
              )
        :contact-move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              )
        ;; (list (send *coords* :get-rleg )
        ;;       (send *coords* :get-lleg )
        ;;       )
                
        :CONTACT-constraint
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
                   :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *coords* :get-rleg) :worldcoords)
                   )
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *coords* :get-lleg) :worldcoords)
                   )

         )
        
        ;; :translation-axis (list t t)
        ;; :rotation-axis (list t t)
        :translation-axis (list :xy :xy t t)
        :rotation-axis (list t t t t)
        ;; :debug-view t
        :root-virtual-mode :6dof
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
                                 #f(300 0 0))

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
(defun send-pose (time &key (force-move nil))
  (objects)
  (when (or force-move (y-or-n-p "Do you move real machine?"))
    (send *ri* :angle-vector (send *robot* :angle-vector) time)
    ;; (send *ri* :wait-interpolation)
    )
  )
(defun reach()
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (send *robot* :reset-manip-pose)
  (send *coords* :set-rleg (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  ;; (send (car (send *robot* :links)) :newcoords (make-coords :pos #f(0 0 600) :rpy (float-vector 0 (deg2rad 50) 0)))

  ;;もどす
  (send *robot* :move-coords (make-coords :pos #f(0 0 600) :rpy (float-vector 0 (deg2rad 50) 0)) (car (send *robot* :links)))
  
  
  (setq rarm-coords (make-coords :pos (float-vector 290 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 290  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))

  ;; (setq rarm-coords (make-coords :pos (float-vector 390 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
  ;; (setq larm-coords (make-coords :pos (float-vector 390  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))

  ;; (setq rleg-coords (make-coords :pos #f(0 -100 0))
  ;;       lleg-coords (make-coords :pos #f(0  100 0)))
  ;;以下を入れるとsegfault
  ;; (setq rleg-coords  (send *coords* :get-rleg)
  ;;       lleg-coords  (send *coords* :get-lleg))

  ;; (send *coords* :set-rleg (make-coords :pos #f(0 -100 0)))
  ;; (send *coords* :set-lleg (make-coords :pos #f(0  100 0)))
  ;; (setq rleg-coords  (send *coords* :get-rleg)
  ;;       lleg-coords  (send *coords* :get-lleg))
  ;; (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog #f(0 0 1000) :move-root-link :z)

  ;;  もどす
  ;; (my-ik rarm-coords larm-coords :cog #f(0 0 1000) :move-root-link :z)
  (ik-walk rarm-coords larm-coords :cog #f(0 0 1000) :move-root-link :z)  
  ;; (ik-st rarm-coords larm-coords :cog #f(0 0 1000) :move-root-link :z)  

;  (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog #f(0 0 1000))

  ;;もどす
  (send-pose movetime :force-move flag-simulator)
  ;; (my-ik #f(0 0 40) #f(0 0 40))

  )

;;持ち上げる
(defun heave
    (rarm-move larm-move &key (cog #f(0 0 700)) (degree 0))
  ;; (send *robot* :move-coords (make-coords :pos #f(0 0 600) :rpy (float-vector 0 (deg2rad 50) 0)) (car (send *robot* :links)))
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm-move))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move))

  ;; needless??
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (setq rarm-coords (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm-move) :rotate (deg2rad degree) :z))
  (setq larm-coords (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move) :rotate (* -1 (deg2rad degree)) :z))
  ;; (setq rarm-coords (make-coords :pos (float-vector 290 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
  ;; (setq larm-coords (make-coords :pos (float-vector 290  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))  
  (setq rleg-coords (make-coords :pos #f(0 -100 0))
        lleg-coords (make-coords :pos #f(0  100 0)))
  ;; (setq rleg-coords  (send *robot* :rleg :end-coords :copy-worldcoords)
  ;;       lleg-coords  (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq rleg-coords  (send *coords* :get-rleg)
  ;;       lleg-coords  (send *coords* :get-lleg))
  
  ;;load した状態でmy-ikを使うとセグフォ
  ;; (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)  
  (ik-walk rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)

  (send *coords* :set-rleg (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (ik-walk rarm-coords larm-coords :cog cog)
  (move-on-foot rleg-coords (send *robot* :lleg :end-coords :copy-worldcoords)   (send *robot* :rleg :end-coords :copy-worldcoords))

  (send-pose movetime :force-move flag-simulator)
  (setq rarm-ref-f (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  (setq larm-ref-f (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  ;; (check-while-still rarm-ref-f larm-ref-f )


  ;; (check rarm-coords larm-coords (list 80 80) 5)

  )

(defun move-on-foot(initial-pos next-pos  nextnext-pos)
  (send *ri* :set-foot-steps
        (list (make-coords :coords initial-pos :name :rleg)
              (make-coords :coords next-pos :name :lleg)
              (make-coords :coords nextnext-pos :name :rleg)
)))
;; (setq waiting nil)
;;静止時の以上検知
;;前の動作終了時の力をref-fとして与えておｓ子から外れると落としたと判断
(defun check-while-still (rarm-ref-f larm-ref-f &key (waiting-time 4) (sleep-time 20000) (thre-list (list 10 10)) (fin 3))
  (setq pre-angle (send *ri* :state :angle-vector))

  (dotimes (/ (* waiting-time 1000000) sleep-time)
    (setq rarm-f  (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
    (setq larm-f  (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
    ;; (setq rarm-dif (norm (v- rarm-ref-f rarm-f) ))
    ;; (setq larm-dif (norm (v- larm-ref-f larm-f) ))
    (setq rarm-dif (abs (elt (v- rarm-ref-f rarm-f) 2)))
    (setq larm-dif (abs (elt (v- larm-ref-f larm-f) 2)))
    (setq drop (list nil nil))
    (format t "rarm-dif = ~A~%" rarm-dif)
    (format t "larm-dif = ~A~%" larm-dif)
    (if (> rarm-dif (elt thre-list 0))
        (setq drop (list-insert t 0 drop)))
    (if (>  larm-dif (elt thre-list 1))
        (setq drop (list-insert t 1 drop)))
    (if (or (elt drop 0) (elt drop 0))  
        (progn
          (print "drop")          
          (return pre-angle)))
    (if (and (< rarm-dif fin) (< larm-dif fin))
        (progn
          (print "goal")
          (return t)))

    (setq pre-angle (send *ri* :state :angle-vector))
    (unix:usleep 20000)
    )
  
  )

;;もとの姿勢に戻すか、正常なハンドの位置に合わせて戻すか
(defun retry-while-still (drop-arm &key (same-pose nil) (cog #f(100 0 0)))
  (send *robot* :angle-vector (send *poses* :get-fin-pose-ri))
  ;;動作終了時
  (if same-pose
      (progn
        (format t "retry to same pose~%")
        (setq rarm-coodrs (send *robot* :rarm :end-coords :copy-worldcoords))
        (setq larm-coodrs (send *robot* :larm :end-coords :copy-worldcoords))
        (my-ik rarm-coords larm-coords :cog cog))
    (progn
      (format t "return to other arm pos")
      (if (eq drop-arm :rarm)
          (setq hold-arm :larm)
        (setq hold-arm :rarm))

      (setq hold-arm-coords (send *robot* hold-arm :end-coords :copy-worldcoords))
      (setq hold-arm-coords-tmp (send hold-arm-coords :copy-worldcoords))
      (setq drop-arm-target-coords (make-coords :pos (aset (send (send hold-arm-coords-tmp :copy-worldcoords) :worldpos) 1 (* -1 (elt (send hold-arm-coords-tmp :worldpos) 1)))  :rpy (car (send hold-arm-coords-tmp :rpy-angle ))))

      (if (eq drop-arm :rarm)
          (my-ik drop-arm-target-coords hold-arm-coords :cog cog)
        (my-ik hold-arm-coords drop-arm-target-coords :cog cog))
      
      ))
  (send-pose movetime :force-move flag-simulator)
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  )
  
  
;;   (setq  (send *robot* :angle-vector))
;;   (setq rarm-coords (send *robot* arm :end-coords))
;;   (unix:sleep 3)
;;   (send *robot* :angle-vector (send *ri* :state :angle-vector))
;;   (setq rarm-coords-drop (send *robot* arm :end-coords))
;;   (my-ik rarm-retry-coords larm-retry-coords :rleg rleg-coords :lleg lleg-coords :cog cog)
;; )

(defun change-arm
    (offset arm)

  
    (send *robot* :angle-vector (send *ri* :state :angle-vector))
    ;; (send *robot* arm :move-end-pos (scale offset #f(-2 0 0)) :local)
    ;;choreonoid
    (send *robot* arm :move-end-pos (scale offset #f(-1 0 0)) :local)
    (send-pose movetime :force-move flag-simulator)

    ;;jikki
    ;; (if (eq arm :rarm)
    ;;     (send *robot* arm :move-end-pos (scale offset #f(0 2 0)) :local)
    ;;   (send *robot* arm :move-end-pos (scale offset #f(0 -2 0)) :local))
    ;;choreonoid
    (if (eq arm :rarm)
        (send *robot* arm :move-end-pos (scale offset #f(0 1.5 0)) :local)
      (send *robot* arm :move-end-pos (scale offset #f(0 -1.5 0)) :local))

    (send-pose movetime :force-move flag-simulator)

    ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
    ;; (if (eq arm :rarm)
    ;;     (send *robot* arm :move-end-pos (scale offset #f(1.5 -1.5 0)) :local)
    ;;   (send *robot* arm :move-end-pos (scale offset #f(1.5 1.5 0)) :local))

    ;;choreonoid
    (if (eq arm :rarm)
        (send *robot* arm :move-end-pos (scale offset #f(1.5 0 0)) :local)
      (send *robot* arm :move-end-pos (scale offset #f(1.5 0 0)) :local))

    (send-pose movetime :force-move flag-simulator)
  )



;;手先位置は固定で腰を高くするheight 1000とか
;;stand-up 1000 :cog #f( 100 0 800)
(defun stand-up
    (height &key (cog #f(0 0 700)) (degree 0) (move-root-link :z))

  (setq rarm-coords-const (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq larm-coords-const (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rleg-coords  (send *robot* :rleg :end-coords :copy-worldcoords)
        lleg-coords  (send *robot* :lleg :end-coords :copy-worldcoords))

  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :reset-manip-pose)
  ;; (send *robot* :reset-pose)

  (setq waist-pos (send (car (send *robot* :links)) :worldpos))
  ;; (aset waist-pos 2 height)
  (setq coo (make-coords :pos (float-vector (elt waist-pos 0) (elt waist-pos 1) height) :rpy (float-vector 0 (deg2rad 40) 0)))
  (send *robot* :move-coords  coo (car (send *robot* :links)))
  ;; (send *robot* :move-coords (make-coords :pos waist-pos :rpy (float-vector 0 (deg2rad 30) 0)) (car (send *robot* :links)))
  (objects)
  ;; (do-until-key (x::window-main-one))
  
  (my-ik rarm-coords-const larm-coords-const :rleg rleg-coords :lleg lleg-coords :cog cog :move-root-link move-root-link :virtual-joint-weight #f(0.5 0.5 0.5 0 0 0))
  ;; (objects)(read-line)
  (send-pose movetime :force-move flag-simulator)
  )
  

;;#f(-10.2571 2.48265 -119.158 121.041 -23.7349 -2.51195 -10.0688 2.94909 -119.142 115.161 -17.879 -3.01482 -5.32426 12.8033 -17.6559 0.0 30.0 29.4087 -133.23 -41.5248 10.8825 -14.2435 111.379 53.7101 -33.4791 -16.9189 -143.345 22.992 5.13361 -7.01039 -144.568 -56.239 -30.3203)
(defun change-arm-developing
    (putback-offset &key (cog #f(0 0 700)) (degree 0) (move-root-link :z))
  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rleg-coords  (send *robot* :rleg :end-coords :copy-worldcoords)
        lleg-coords  (send *robot* :lleg :end-coords :copy-worldcoords))


  ;;put back r-hand
  ;; (setq rarm-coords-putback (send rarm-coords :translate (scale (* -1 putback-offset) (send  (send (send *robot* :rarm :end-coords) :copy-worldcoords) :x-axis))))
  ;; (setq rarm-coords-putback (send (send *robot* :rarm :end-coords :copy-worldcoords) :transform-vector (scale (* -1 putback-offset) #f(1 0 0)) ))
  (setq rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rarm-coords-putback (send (send rarm-coords :copy-worldcoords) :translate (v- (send (send rarm-coords :copy-worldcoords) :transform-vector (scale putback-offset #f(-1 0 0))) (send rarm-coords :worldpos))))
  (my-ik rarm-coords-putback larm-coords :rleg rleg-coords :lleg lleg-coords  :move-root-link move-root-link)
  (send-pose movetime :force-move flag-simulator)

  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (setq rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rarm-coords-slide (send (send rarm-coords :copy-worldcoords) :translate (v- (send (send rarm-coords :copy-worldcoords) :transform-vector (scale putback-offset #f(0 1 0))) (send rarm-coords :worldpos))))
  (my-ik rarm-coords-slide larm-coords :rleg rleg-coords :lleg lleg-coords :move-root-link move-root-link)
  (send-pose movetime :force-move flag-simulator)

  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (setq rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rarm-coords-push (send (send rarm-coords :copy-worldcoords) :translate (v- (send (send rarm-coords :copy-worldcoords) :transform-vector (scale putback-offset #f(1 0 0))) (send rarm-coords :worldpos))))
  (my-ik rarm-coords-push larm-coords :rleg rleg-coords :lleg lleg-coords  :move-root-link move-root-link)
  (send-pose movetime :force-move flag-simulator)

  
  ;; (setq rarm-coords-slide (send rarm-coords :translate (scale (* -1 putback-offset) (send  (send (send *robot* :rarm :end-coords) :copy-worldcoords) :x-axis))))
  ;; (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog :move-root-link move-root-link)

  )

(defun arm
    (putback-offset)
    (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rleg-coords  (send *robot* :rleg :end-coords :copy-worldcoords)
        lleg-coords  (send *robot* :lleg :end-coords :copy-worldcoords))

  (send *robot* :rarm :move-end-pos (v- (send (send rarm-coords :copy-worldcoords) :transform-vector (scale putback-offset #f(-1 0 0))) (send rarm-coords :worldpos)))
  (send-pose movetime :force-move flag-simulator)

  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (setq rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (send *robot* :rarm :move-end-pos (v- (send (send rarm-coords :copy-worldcoords) :transform-vector (scale putback-offset #f(0 1 0))) (send rarm-coords :worldpos)))
  (send-pose movetime :force-move flag-simulator)

  )



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

(defun push-board ()
  (if auto-detect-p
      (send *ri* :set-object-turnaround-ref-force :max-ref-force (* 9.8 6.0))
    (progn
      (send *ri* :set-ref-force (float-vector 0 0 (* 9.8 2.5 -1)) 2000 :arms)
      (send *ri* :wait-interpolation-seq))))
  
 
;;   (my-ik rarm-coords larm-coords)
;;   (send-pose 3000 :force-move nil)
;; )  
  

(defun prepare-ex ()
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st))

  )

(defun log (name ang)
  (send *ri* :start-log)
  (unix:sleep 3)
  (my-init-pose)
  (unix:sleep 2)
  (reach)
  (unix:sleep 2)
  
  (save-log :fname name)
  )

(defun my-init-pose()
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (objects)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :reset-manip-pose)
  (send-pose movetime :force-move flag-simulator)
  )

;; (defun save(&key (fname "emergency"))
;;   ;; (setq path "~/google-drive/JSK/log")
;;     (setq path "/home/utaro/google-drive/JSK/log")
                 
;;   (send *ri* :save-log (format nil "~A/~A" path name) :make-directory t)
;; )
(print "my-init-pose")
(print "(reach #f(260 0 -700) #f(260 0 -700))")

;;(prepare-ex)


;; rarm
;; #f(80.0 90.0 -100.0 0.0 0.5 -12.0 0.0 0.0)
;; larm
;; #f(-81.4058 90.0 100.0 0.0 0.0 0.0 0.0 0.0)

;;rarm
;; #f(80.0 0.0 -92.5 0.0 -10.0 -12.0 0.0 0.0)
;; larm
;; #f(-80.0 0.0 91.7 0.0 -10.0 0.0 0.0 0.0)


;; (* (* (/ data 360) 
      ;; ) 


;;;(progn (load "~/research/rm_debris/rm_board/euslisp/heave.l") (prepare-simulator) (my-init-pose) (reach) (heave #f(100 0 0) #f( 100 0 0)) (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500) (send *ri* :set-ref-force (float-vector 0 0 (* 9.8 4 -1)) 2000 :arms) (heave #f(0 -50 0) #f(0 50 0)))
;;heave #f(0 -200 0) #f(0 200 0)
;;(send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)

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

  (progn (setq theta 80) (send *ri* :set-ref-force (scale 2 (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta))))) 2000 :larm) (change-arm 140 :rarm))
  (progn (setq theta 80) (send *ri* :set-ref-force (scale 2 (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta))))) 2000 :rarm) (change-arm 140 :larm))

  ;; (progn (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))

  (send *ri* :go-pos 0.3 0 0)
  (send *ri* :go-pos 0.3 0 0)

  ;; (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms)
  
  )

(defun dem-pre()
  (my-init-pose)
  (unix::sleep 3)
  (reach)
  (unix::sleep 3)
)

(defun dem-mid()  
  ;; (heave #f(100 0 0) #f( 100 0 0) :cog #f(100 0 700))
  ;; (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)

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
  
  (stand-up 1000  :cog #f(100 0 900))
  (unix::sleep 3)
  
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
