(require "models/arrow-object.l")
(require "package://control_tools/log-tools.l")
;; (require "package://control_tools/play-hrpsysfile.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;; (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "./class.l")
(load "./my-ik.l")
;; (require "../../euslisp/save-log.l")
;;平面認識使わないときはcomment out
;; (load "./client/make_plane_for_opt.l")
;; (require "~/research/rm_debris/src/predict_from_multi_pose.l")

(print "load functions.l")

(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
 )
(setq movetime 16000)
;; (objects (list *robot*))
(unless (boundp 'flag-simulator)
  (setq flag-simulator nil) ;;simulatorではｔ
  (setq movetime 16000))


(setq *coords* (instance my-coords :init
                       (copy-object (send *robot* :rarm :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :larm :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :rleg :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :lleg :end-coords :copy-worldcoords))
                       #f(0 0 700)
                       ))

(setq *poses* (instance poses :init
                       (send *robot* :angle-vector)
                       (send *ri* :state :angle-vector)))

(defun prepare-simulator()
  (setq flag-simulator t) ;;simulatorではｔ
  (setq movetime 4000)
  (send *ri* :start-auto-balancer :limbs (list :lleg :rleg :larm :rarm))
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :set-log-maxlength (* 500 60))
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st))
  )

(defun prepare-real-machine()
  (send *ri* :start-auto-balancer :limbs (list :lleg :rleg :larm :rarm))
  (setq flag-simulator nil)
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st))
  (send *ri* :set-log-maxlength (* 500 60))
  (setq movetime 16000)
  )
(defun send-pose
    (time &key (pre-pose (send *robot* :angle-vector)) (pre-lleg  (send *robot* :lleg :end-coords :copy-worldcoords) ) (force-move nil) (wait? t) (resume? nil) (look-at-object? nil))
  (if look-at-object?
      (send *robot* :head :look-at (v+ (midpoint 0.5 (send (send *robot* :rarm :end-coords :copy-worldcoords) :worldpos) (send (send *robot* :larm :end-coords :copy-worldcoords) :worldpos)) #f(500 0 100))))
  (objects)
  (if (or force-move (setq ans (y-or-n-p "Do you move real machine?")))
      (progn
        (send *ri* :angle-vector (send *robot* :angle-vector) time)
        
        (if wait?
            (progn
              (print "wait-interpolation")
              (send *ri* :wait-interpolation)))
        )
    (progn
      (if resume?
	  (progn
	    (send *robot* :angle-vector pre-pose)
	    (send *robot* :move-coords (send pre-lleg :translate #f(0 0 140)) (send *robot* :link "LLEG_LINK5"))
	    (objects)
	    (print "pre pose"))
      ))
    ))

(defun reach
    (&key (fix-contact? nil)
          (send? t)
          (wait? t)
          (avoid-collision? nil)
          (apply-external-force? nil)
          (external-force-list  (list (float-vector 0 0 -80 0 0 0)
                                      (float-vector 0 0 -80 0 0 0)))
          ;; (external-coords-list (list (send *robot-env* :robot :get :rarm-contact-coords)
          ;;                             (send *robot-env* :robot :get :larm-contact-coords)))
          (rarm-coords (make-coords :pos (float-vector 390 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
          (larm-coords (make-coords :pos (float-vector 390  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))
          (rleg-pos-shift #f(0 0 -10))
          (lleg-pos-shift #f(0 0 -10))
          (look-at-object? nil)
          )
  (send *robot* :reset-manip-pose)
  (send *coords* :set-rleg (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg (send *robot* :lleg :end-coords :copy-worldcoords))

  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (ik-opt rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :external-force-list external-force-list ;; :external-coords-list external-coords-list
          )

  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
;;   (setq zmp-vector (send *ri*  :zmp-vector))
;;   (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
;;   (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
;;   (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
;;   (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
;;   (if send?
;;       (progn 
;;         (if (not fix-contact?)
;;             (progn
;;               (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
;;               (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t)))
;; ;  (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog #f(0 0 1000))

;;         ;;もどす
;;         (send-pose movetime :force-move flag-simulator :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?  )))
  ;; (my-ik #f(0 0 40) #f(0 0 40))
  )

;;持ち上げる
(defun heave
    (rarm-move larm-move
               &key  (fix-contact? t) (send? t) (cog #f(0 0 700))
               (deg-r-x 0) (deg-r-y 0) (deg-r-z 0) (deg-l-x 0) (deg-l-y 0) (deg-l-z 0)
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 0))
               (lleg-pos-shift #f(0 0 0))
               (look-at-object? nil))

  (setq rarm-coords (send (send (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm-move) :rotate (deg2rad deg-r-x) :x) :rotate (deg2rad deg-r-y) :y) :rotate (deg2rad deg-r-z) :z))
    (setq larm-coords (send (send (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move) :rotate (deg2rad deg-l-x) :x) :rotate (deg2rad deg-l-y) :y) :rotate (deg2rad deg-l-z) :z))

  (setq rleg-coords (make-coords :pos #f(0 -100 0))
        lleg-coords (make-coords :pos #f(0  100 0)))

  (setq rleg-coords-old (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords-old (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (ik-opt rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :reset? reset? :apply-external-force? apply-external-force? :avoid-collision? avoid-collision? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift)

  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             (move-on-foot rleg-coords-old lleg-coords-old rleg-coords-new lleg-coords-new :wait? nil)
              
  ;;             ))
  ;;       (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?)
  ;;       ))
  )

(defun change-arm
    (offset arm &key (send? t) (sync? t))

  (if sync?
     (send *robot* :angle-vector (send *ri* :state :angle-vector))) 
  ;;腕を引く
  (send *robot* arm :move-end-pos (scale offset #f(-1 0 0)) :local)
  (if send?
      (progn
        (send-pose (- movetime 1500) :force-move flag-simulator )))

  ;; (send *ri* :start-grasp arm)
  (send *ri* :move-gripper arm 0.4 :effort 10 :wait t) 
  ;;腕を下へ
  (if (eq arm :rarm)
      (send *robot* arm :move-end-pos (scale offset #f(0 1.4 0)) :local)
    (send *robot* arm :move-end-pos (scale offset #f(0 -1.4 0)) :local))
  (if send?
      (progn
        (send-pose (- movetime 1500) :force-move flag-simulator )))

  ;;腕を戻す    
  (if (eq arm :rarm)
      (progn
        (send *robot* :larm :move-end-pos (scale offset #f(0 0.2 0)) :local)
        (send *robot* arm :move-end-pos (scale offset #f(1.7 0 0)) :local))
    (progn
      (send *robot* :rarm :move-end-pos (scale offset #f(0 -0.2 0)) :local)
    (send *robot* arm :move-end-pos (scale offset #f(1.7 0 0)) :local)))
  (if send?
      (progn
        (send-pose (- movetime 1500) :force-move flag-simulator )))
  
  )

(defun move-hand
    (arm relative-pos &key (time 8000) (send? t) (sync? nil))
  (if sync?
      (send *robot* :angle-vector (send *ri* :state :angle-vector)))
  (send *robot* arm :move-end-pos relative-pos :local)
  (if send?
      (progn
        (send-pose time :force-move flag-simulator )))
  )
(defun move-on-foot
    (init-rcoords init-lcoords next-rcoords  next-lcoords &key (which? :lleg) (only-move? nil) (wait? t) (force-move nil))
  (format t "init-lcoords = ~A~%" init-lcoords)
  (format t "next-rcoords = ~A~%" next-rcoords)
  (format t "next-lcoords = ~A~%" next-lcoords)
  ;; (if (setq ans (y-or-n-p "move foot?"))
  (if (or force-move (setq ans (y-or-n-p "Do you move foot?")))
      (if (eq which? :lleg)
          (if wait?
              (send *ri* :set-foot-steps
                    (list
                     (make-coords :coords init-rcoords :name :rleg)
                     (make-coords :coords  next-lcoords :name :lleg)
                     (make-coords :coords  next-rcoords :name :rleg)
                     ))
            (send *ri* :set-foot-steps-no-wait            
                  (list
                   (make-coords :coords init-rcoords :name :rleg)
                   (make-coords :coords  next-lcoords :name :lleg)
                   (make-coords :coords  next-rcoords :name :rleg)
                   
                   )))
        (if wait?
            (send *ri* :set-foot-steps
                  (list
                   (make-coords :coords init-rcoords :name :rleg)
                   (make-coords :coords  next-lcoords :name :lleg)
                   (make-coords :coords  next-rcoords :name :rleg)
                   ))
          (send *ri* :set-foot-steps-no-wait
                (list
                 (make-coords :coords init-lcoords :name :lleg)
                 (make-coords :coords  next-rcoords :name :rleg)
                 (make-coords :coords  next-lcoords :name :lleg)
                 
                 ))
          ))
    )
  (if only-move? 
      (send *robot* :angle-vector (send *ri* :state :angle-vector)))
  (send *robot* :move-coords (send (send next-lcoords :copy-worldcoords) :translate #f(0 0 100)) (send *robot* :link "LLEG_LINK5"))
  (send *robot* :move-coords (send (send next-rcoords :copy-worldcoords) :translate #f(0 0 100)) (send *robot* :link "RLEG_LINK5"))
  (objects)
  (send *ri* :angle-vector (send *ri* :potentio-vector))
  )


(defun dive
    (rarm-move larm-move
               &key
               rarm-coords
               larm-coords
               (fix-contact? t) (send? t) (cog #f(0 0 700))
               (deg-r-x 0) (deg-r-y 0) (deg-r-z 0) (deg-l-x 0) (deg-l-y 0) (deg-l-z 0)
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 -10))
               (lleg-pos-shift #f(0 0 -10))
               (look-at-object? nil)
               (face (cube 800 600 20))
               )

  (if (not rarm-coords)
      (setq rarm-coords (send (send (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm-move) :rotate (deg2rad deg-r-x) :x) :rotate (deg2rad deg-r-y) :y) :rotate (deg2rad deg-r-z) :z)))
  (if (not larm-coords)
      (setq larm-coords (send (send (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move) :rotate (deg2rad deg-l-x) :x) :rotate (deg2rad deg-l-y) :y) :rotate (deg2rad deg-l-z) :z)))

  (setq rleg-coords-old (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords-old (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (ik-opt-with-larmcontact rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :reset? reset? :apply-external-force? apply-external-force? :avoid-collision? avoid-collision? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :face face)
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn
  ;;       ;; (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg)
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             (move-on-foot rleg-coords-old lleg-coords-old rleg-coords-new lleg-coords-new :wait? nil))
  ;;             )
  ;;       (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?)
  ;;       ))
  )

(defun dive-larm-only
    ( larm-move
               &key
               larm-coords
               (fix-contact? t) (send? t) (cog #f(0 0 700))
               (deg-r-x 0) (deg-r-y 0) (deg-r-z 0) (deg-l-x 0) (deg-l-y 0) (deg-l-z 0)
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 -10))
               (lleg-pos-shift #f(0 0 -10))
               (look-at-object? nil))

  ;; needless??
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (if (not larm-coords)
      (setq larm-coords (send (send (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move) :rotate (deg2rad deg-l-x) :x) :rotate (deg2rad deg-l-y) :y) :rotate (deg2rad deg-l-z) :z)))

  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (ik-opt-with-larmcontact-only larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :reset? reset? :apply-external-force? apply-external-force? :avoid-collision? avoid-collision? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift)
  (if send?
      (progn
        (if (not fix-contact?)
            (progn
              (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
              (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords)   (send *robot* :lleg :end-coords :copy-worldcoords) :wait? nil)))
        (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?)
        ))
  )
(setq flag-pcl 0)
;;output flag   0 ::both arms holding , 1 ::larm drop , 2 ::rarm drop , 3 ::both arms drop, 10 ::finish smoothly
(defun check-force-pcl
    (&key (checking-time 6) (sleep-time 20000) (working? t) (thre-list (list 10 10)) (fin 3))
  (setq initial (send *robot* :angle-vector)
        rarm-f-pre  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil)
        larm-f-pre  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil)
        angle-pre (send *ri* :state :angle-vector)
        flag 0)

  (if working? 
      (format t "~%~%check while working~%")
    (format t "~%~%check while still~%"))
  
  (dotimes (i (/ (* checking-time 1000000) sleep-time))
    (setq flag 0)
    (format t "~%~%")
    (setq rarm-f  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil))
    (setq larm-f  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil))
    ;;絶対値バージョン
    (setq rarm-f-dif (abs (elt (v- rarm-f rarm-f-pre) 2)))
    (setq larm-f-dif (abs (elt (v- larm-f larm-f-pre) 2)))
    ;; (setq rarm-f-dif (elt (v- rarm-f rarm-f-pre) 2))
    ;; (setq larm-f-dif (elt (v- larm-f larm-f-pre) 2))
    (setq drop (list nil nil))
    (format t "rarm-f-dif       = ~A~%" rarm-f-dif)
    (format t "larm-f-dif       = ~A~%" larm-f-dif)

    ;;drop判定 ;;へんこうする
    (if (>  larm-f-dif (* +1 (elt thre-list 1)))
        (print "larm dif")
        (setq flag (+ flag 1)))
    (if (>  rarm-f-dif (* +1 (elt thre-list 0)))
        (setq flag (+ flag 2)))
    (format t "flag = ~A~%" flag)
    (setq flag-pcl (check-pcl))
    (format t "flags = ~A ~A ~%" flag flag-pcl)
    (if (or (not (= 0 flag-pcl))  (not (= flag 0)))
        (if (= flag-pcl 0)
            (print "holding")
          (progn
            (cond 
             ((= flag-pcl 1)
              (print "drop larm"))
             ((= flag-pcl 2)
              (print "drop rarm"))
             ((= flag-pcl 3)
              (print "drop both arm")))
            (return-from check-force-pcl flag-pcl))))
      
    (if working?
        (progn
          ;;動作終了判定
          (setq tmp (send *ri* :reference-vector))
          (setq angle-vector-dif (abs (norm (v- initial tmp))))
          (format t "angle-vector-dif = ~A~%" angle-vector-dif)
          (if (< angle-vector-dif 0.5)
              (progn
                (print "finish")
                (setq flag 10)
                (return angle-vector-dif))
            )
          (setq rarm-f-pre rarm-f)
          (setq larm-f-pre larm-f)))
    
    (setq angle-pre (send *ri* :state :angle-vector))
    
    (unix:usleep sleep-time)
    )
  (format t "flag = ~A~%" flag)
  flag
  )
(setq drop_flag_larm (instance std_msgs::Bool :init))
(setq drop_flag_rarm (instance std_msgs::Bool :init))
(send drop_flag_larm :data nil)
(send drop_flag_rarm :data nil)
;;output flag   0 ::both arms holding , 1 ::larm drop , 2 ::rarm drop , 3 ::both arms drop, 10 ::finish smoothly
(defun check-pcl()
  (let*
      ((drop_flag_larm (one-shot-subscribe "/drop_flag_larm" std_msgs::Bool :timeout 3000))
       (drop_flag_rarm (one-shot-subscribe "/drop_flag_rarm" std_msgs::Bool :timeout 3000))
       (flag 0))
    (format t "get larm points = ~A~%" (send drop_flag_larm :data))
    (format t "get rarm points = ~A~%" (send drop_flag_rarm :data))
    (if (not (send drop_flag_larm :data))
        (setq flag 1))
    (if (not (send drop_flag_rarm :data))
        (setq flag 2))
    (if (and (not (send drop_flag_larm :data)) (not (send drop_flag_rarm :data)))
        (setq flag 3))
    flag)
  )


;;もとの姿勢に戻すか、正常なハンドの位置に合わせて戻すか
(defun retry-while-working (drop-arm &key (offset 140) (same-pose nil) (cog #f(100 0 0)))
  ;;一度手を引っ込める
  (if (eq drop-arm :rarm)
      (progn
        (heave (scale offset #f(-1 0.5 0)) #f(0 0 0))
        )
    (heave #f(0 0 0) (scale offset #f(-1 0.5 0) )))
  (print "pull back once")
  (send-pose movetime :force-move flag-simulator)
  
  ;;ロボットモデルを新しく作りriの現在姿勢をとる
  (setq *rbt* (instance jaxon_red-robot :init))
  (send *rbt* :angle-vector (send *ri* :state :angle-vector))
  (send *rbt* :fix-leg-to-coords (make-coords))
  (objects (list *robot* *rbt*))
  
  (setq rleg-coords (make-coords :pos #f(0 -100 0))
        lleg-coords (make-coords :pos #f(0  100 0)))

  ;;動作終了時
  ;;もとの姿勢に戻るかどうか
  (if same-pose
      (progn
        (format t "retry to same pose~%")
        (setq rarm-coodrs (send *rbt* :rarm :end-coords :copy-worldcoords))
        (setq larm-coodrs (send *rbt* :larm :end-coords :copy-worldcoords))
        (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog))
    (progn
      (format t "return to other arm's pos~%")
      (if (eq drop-arm :rarm)
          (progn
            (setq hold-arm :larm)
            (setq drop-arm-offset #f(50 50 0)))
        (progn
          (setq hold-arm :rarm)
          (setq drop-arm-offset #f(50 -50 0))))
      (format t "regrasp ~A ~%" drop-arm)
      (setq hold-arm-coords (send *robot* hold-arm :end-coords :copy-worldcoords))
      (send hold-arm-coords :translate #f(50 0 0) :local)
      (format t "hold-arm-coords = ~A~%" hold-arm-coords)
      ;;現在姿勢を利用（実機では余り腕がたれないかもその時はriを使う必要はない
      (setq hold-arm-coords-tmp (send *rbt* hold-arm :end-coords :copy-worldcoords))
      (format t "hold-arm-coords-tmp = ~A~%" hold-arm-coords-tmp)
      ;;y成分の符号を反転
      (aset (send hold-arm-coords-tmp  :worldpos) 1 (* -1 (elt (send hold-arm-coords-tmp :worldpos) 1)))
      ;;rpyのx，z成分の符号反転
      (setq rpy-tmp (float-vector (* -1 (elt (car (send (send hold-arm-coords-tmp :worldcoords) :rpy-angle)) 0 )) (elt (car (send (send hold-arm-coords-tmp :worldcoords) :rpy-angle)) 1 ) (* -1 (elt (car (send (send hold-arm-coords-tmp :worldcoords) :rpy-angle)) 2 ))))

      (setq drop-arm-target-coords (make-coords :pos (send hold-arm-coords-tmp :worldpos)  :rpy rpy-tmp))
      (send drop-arm-target-coords :translate drop-arm-offset :local)
      (format t "drop-arm-target-coords = ~A~%" drop-arm-target-coords)

      
      (if (eq drop-arm :rarm)
          (progn
            (my-ik drop-arm-target-coords hold-arm-coords :rleg rleg-coords :lleg lleg-coords :cog cog))
            (progn
              (my-ik hold-arm-coords drop-arm-target-coords :rleg rleg-coords :lleg lleg-coords :cog cog)))
      ))
  (send-pose movetime :force-move flag-simulator)

  )

(defun retry-flag-checker
    (flag)
  (cond
   ((eq 1 flag)
    (retry-while-working :rarm ))
   ((eq 2 flag)
         (retry-while-working :larm ))
   (t
    (print "do nothing"))))

(defun start-coords()
  (send *robot* :move-coords (make-coords :pos  #f(0 100 0)) (send *robot* :lleg :end-coords))
  ;; (send *robot* :move-coords (make-coords :pos #f(0 100 100) :rpy #f(0 0 0) ) (send *robot* :link "LLEG_LINK5"))
  (objects))

  
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

(defun push-board ()
  (if auto-detect-p
      (send *ri* :set-object-turnaround-ref-force :max-ref-force (* 9.8 6.0))
    (progn
      (send *ri* :set-ref-force (float-vector 0 0 (* 9.8 2.5 -1)) 2000 :arms)
      (send *ri* :wait-interpolation-seq))))
  
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

(defun my-init-pose
    (&key (move-foot? nil) (only-upper? nil) (send? t))
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (objects (list *robot*))
  (print "init pose ")
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (if only-upper?
      (progn
        (setq reset-pose-upper #f(0.0 0.0 0.0 0.0 30.0 0.0 55.0 -20.0 -15.0 -100.0 -25.0 0.0 -45.0 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
        (setq pose-under (subseq (send *robot* :angle-vector) 0 12))
        (send *robot* :angle-vector (concatenate float-vector pose-under reset-pose-upper))
        (send *robot* :move-coords (send lleg-coords :translate #f(0 0 100)) (send *robot* :link "LLEG_LINK5")))
    (progn
      (send *robot* :reset-manip-pose)
      (start-coords)))
  (if send? 
      (send-pose movetime :force-move flag-simulator))
  ;; (send *ri* :hand-reset) ;;実機では必要？choreonoidでは不要？
  (format t "left or right ?  ~A" (elt (send (car (send *robot* :torso)) :worldpos) 1))
  (if move-foot?
      (if (< 0 (elt (send (car (send *robot* :torso)) :worldpos) 1))
          (progn
            (start-coords)
            (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) :which? :lleg))
        (progn
          (start-coords)
          (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) :which? :rleg))
        )
    )
  )

(defun average
    (lis &key  (axis 10))
  (let* ((len (list-length lis))
         ;; (sum (list 0 0 0)))
         (sum_x 0)
         (sum_y 0)
         (sum_z 0))
         
    (dotimes (i len)
      (setq sum_x (+ sum_x (elt (elt lis i) 0)))
      (setq sum_y (+ sum_y (elt (elt lis i) 1)))
      (setq sum_z (+ sum_z (elt (elt lis i) 2)))      
      )
        
    (setq ave (float-vector (/ sum_x (float len)) (/ sum_y (float len)) (/ sum_z (float len))))

    ;; (print (format nil "ave = ~A" ave))
    ;; (print len)
    (cond
     ((equal axis 0)
      (float (elt ave 0)))
     ((equal axis 1)
      (elt ave 1))
     ((equal axis 2)
      (float (elt ave 2)))
     (t (progn ;;(print "output vector")
               ave))
     )
    )
  )

(defun low-pass-online
    (target coords arm &key (times 40) (r 0.3) (sleep 10000) (flag nil))
  (let* (
         (y-data (list (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm)))
         (y-predict (list (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm)))
         )
    
    (dotimes (i times)
           (setq data (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm))
      (setq y (v+ (scale r data) (scale (- 1 r) (elt y-predict i))))
      (list-insert y (+ i 1) y-predict)
      (list-insert data (+ i 1) y-data)
      ;; (unix:usleep 1000)
      (unix:usleep sleep)      
      )
    (setq ret (cdr y-predict))
;;    (print (format nil "~A vector through filter = ~A" target ret))
    (setq ret-average (average ret))

    (if flag
        (list  (cdr y-data) ret)
      (progn
        (print (format nil "~A vector through filter and average = ~A" target ret-average))
        ret-average)
    )
    )
  )

;; (setq pre-angle (send *robot* :reset-manip-pose))
;; (setq pre-lleg (make-coords :pos #f(0 -100 0)))
(defun set-pre-pose()
  (setq pre-angle (send *robot* :angle-vector))
  (pre-lleg  (send *robot* :lleg :end-coords :copy-worldcoords))
  (list pre-angle pre-lleg))

(defun resume (pre-angle pre-lleg)
  (send *robot* :angle-vector pre-angle)
  (send *robot* :move-coords (send pre-lleg :translate #f(0 0 0)) (send *robot* :lleg :end-coords))
  (load "change_arm_with_ik.l")
  (prepare-simulator))

(defun quick-sort-vector-list(seq)
  (if (< (length seq) 2)
      (return-from quick-sort-vector-list seq))
  (let ((pivot (elt (elt seq 0) 0))
        (pivot-vec (elt seq 0) )
        (left nil)
        (right nil))

  (dotimes (i (- (length seq) 1))
    (if (< (elt (elt seq (+ i 1)) 0) pivot)
        (setq left (list-insert (elt seq (+ i 1)) (length seq) left))
      (setq right (list-insert (elt seq (+ i 1)) (length seq) right))
      ))
  ;; (format t "left = ~A~%" left)
  ;; (format t "right = ~A~%" right)
  (setq left (quick-sort-vector-list left))
  (setq right (quick-sort-vector-list right))
  (setq fwd (append left (list pivot-vec)))
  (setq ret (append fwd right ))
  ret
  ))

;;listのうちx要素が小さいものをnumこ取り出す
(defun get-min-val(lis num)
  (setq ret-lis (quick-sort-vector-list lis))
  (subseq ret-lis 0 num))

;;plane　オブジェクト(prismなど）をコピーして渡す
;;掴みに行く目標座標
(defun get-target-pose
    (contact-wall arm &key (shift 320))
  (let* ((idx-x 0)
        (idx-y 0)
        (longest-len 0)
        (vertices-list (get-min-val (send contact-wall :vertices) 20))
        (vertices-list-reverse (reverse vertices-list))
        (plane-normal (send (elt (send contact-wall :faces) 0) :normal)))
    (format t "plane normal = ~A~%" plane-normal)
    (dotimes (i (length vertices-list))
      (dotimes (j (- (length vertices-list) (+ 1 i)))
        (setq len-tmp (norm (v- (elt vertices-list i) (elt vertices-list-reverse j))))
        (if (< longest-len len-tmp)
            (progn
              (setq idx-x i)
              (setq idx-y j) ;;reverseされているがそのままいく
              (setq longest-len len-tmp)))))
    (setq longest-edge (make-line (elt vertices-list idx-x) (elt vertices-list-reverse idx-y)))
    (format t "longest-len = ~A~%" longest-len)
    ;; (format t "vertice = ~A~%" (elt vertices-list idx-x))
    ;; (format t "vertice = ~A~%" (elt vertices-list-reverse idx-y))  
    (setq normal-line (make-line #f(0 0 0) plane-normal))
    
    ;;手先の目標
    (setq sum #f(0 0 0))
    (dotimes (i (length vertices-list))
      (setq sum (v+ sum (elt vertices-list i))))
    (setq pos (scale (/ 1.0 (length vertices-list)) sum))
    (format t "mid-pos = ~A~%" pos)
    (setq y-vec  plane-normal)
    (setq z-vec (v- (elt vertices-list idx-x) (elt vertices-list-reverse idx-y)))
    (setq x-vec (v* y-vec z-vec))
    ;;x-vecは正面方向
    (if (< (elt x-vec 0) 0)
        (setq x-vec (scale -1 x-vec)))

    (if (equal arm "larm")
        (progn
          (setq pos (v+ pos (float-vector 0 shift 0)))
          (if (< (elt y-vec 2) 0)
              (setq y-vec (scale -1 y-vec)))
          (if (> (elt z-vec 1) 0)
              (setq z-vec (scale -1 z-vec))))
      (progn
        (setq pos (v+ pos (float-vector 0 (- 0 shift) 0)))
        (if (> (elt y-vec 2) 0)
            (setq y-vec (scale -1 y-vec)))
        (if (< (elt z-vec 1) 0)
            (setq z-vec (scale -1 z-vec)))))

    (setq x-vec (normalize-vector x-vec))
    (setq y-vec (normalize-vector y-vec))
    (setq z-vec (normalize-vector z-vec))
    ;; (format t "plane-normal = ~A~%" plane-normal)
    ;; (format t "x-vec = ~A~%" x-vec)
    ;; (format t "y-vec = ~A~%" y-vec)
    ;; (format t "z-vec = ~A~%" z-vec)
    (setq rot (transpose
               (matrix x-vec y-vec z-vec)))
    (format t "pos = ~A~%" pos)
    (format t "rot = ~A~%" rot)
    (list pos rot)
    ))

;;掴みに行く目標座標　微調整
(defun auto-set-reaching-targets(&key (larm-shift #f(-200 -50 0)) (rarm-shift #f(-200 50 0)))
  (setq cu (make_plane))
  (if (not cu)
      (progn
        (print "can not find any plane")
        (return-from auto-set-reaching-targets nil)))
  (setq obj-larm (copy-object cu))
  (setq obj-rarm (copy-object cu))
  (setq ret-larm (get-target-pose obj-larm "larm"))
  (setq ret-rarm (get-target-pose obj-rarm "rarm"))

  (setq coords1 (make-coords))
  (setq larm-coords (make-coords :pos (elt ret-larm 0) :rpy (elt (rpy-angle (elt ret-larm 1)) 0)))
  (setq rarm-coords (make-coords :pos (elt ret-rarm 0) :rpy (elt (rpy-angle (elt ret-rarm 1)) 0)))
  
  ;;少し回転
  (setq larm-coords (send larm-coords :rotate (deg2rad -20) :z))
  (setq rarm-coords (send rarm-coords :rotate (deg2rad 20) :z))

  (setq larm-coords (send larm-coords :translate larm-shift))
  (setq rarm-coords (send rarm-coords :translate rarm-shift))
  (format t "ik larm-coords = ~A~%" larm-coords)
  (format t "ik rarm-coords = ~A~%" rarm-coords)
  (objects (list cu coords1 larm-coords rarm-coords))
  (list larm-coords rarm-coords)
  )
(defun quick-sort(seq)
    (if (< (length seq) 1)
      (return-from quick-sort seq))
  (let ((pivot (elt seq 0))
        ( left nil)
        ( right nil))

  (dotimes (i (- (length seq) 1))
    ;; (print i)
    (if (< (elt seq (+ i 1)) pivot)
        (setq left (list-insert (elt seq (+ i 1)) (length seq) left))
      (setq right (list-insert (elt seq (+ i 1)) (length seq) right))
      ))
  (format t "left = ~A~%" left)
  (format t "right = ~A~%" right)
  (setq left (quick-sort left))
  (setq right (quick-sort right))
  (setq fwd (append left (list pivot) ))
  (setq ret (append fwd right ))
  ret
  ))

(defun get-half-plane(obj &key (cut-rate 0.5))
  (setq origin-obj (copy-object obj))
  (setq origin-vertices (send (car (send origin-obj :faces)) :vertices))
  (setq com (send origin-obj :centroid))
  (format t "com = ~A~%" com)
  (setq len-lis (length origin-vertices))
  (setq cub (make-cube (* (/ 1.0 cut-rate) (abs (elt com 0))) 5000 5000))
  (send cub :translate (scale 2.0 com))
  (setq sub (body- origin-obj cub))
  (send sub :set-color (float-vector 1 0 0))  
  (objects (list  sub ))
  sub)

(defun board-holding-pose(&key (send? nil) (fix-contact? t) (look-at-object? nil))
  ;; (send *robot* :reset-manip-pose)
  ;; (start-coords)
  (send *ri* :start-impedance :arms)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))  
  (my-init-pose :move-foot? nil :send? nil)
  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (send *robot* :angle-vector  #f(-2.98947 -11.1725 -59.0636 8.42639 24.4303 12.4009 7.00572 10.9221 -59.3257 7.19791 25.9469 -11.8133 0.570165 3.0 1.84699 0.603257 15.0 37.0 35.8508 -131.711 -28.0764 -23.8558 -27.5811 24.8872 -0.448495 -37.0 35.6619 131.041 28.564 -24.874 27.783 -24.4164 0.215248))
  ;; (send *robot* :angle-vector
  ;; 	#f(-1.32255 -6.24016 -29.6795 33.8312 -11.7631 7.52237 0.898833 15.4077 -26.725 24.6561 -5.76486 -14.2255 1.15928 3.0 -0.655261 2.4144 16.5573 37.0 45.5976 -109.064 -30.8867 -64.1641 -26.6447 9.51422 22.1128 -37.0 41.1018 104.469 28.6239 -56.9815 20.0663 -14.0334 11.5481))
  ;; (send *robot* :angle-vector  #f(-1.32255 -6.24016 -29.6795 33.8312 -11.7631 7.52237 0.898833 15.4077 -26.725 24.6561 -5.76486 -14.2255 1.15928 3.0 -0.655261 2.4144 0.0 37.0 45.5976 -109.064 -30.8867 -64.1641 -26.6447 9.51422 22.1128 -37.0 41.1018 104.469 28.6239 -56.9815 20.0663 -14.0334 11.5481))
  ;;from reaching-pose (heave #f(-400 -1000 0) #f(-400 1000 0) :reset? t :fix-contact? 2 :wait? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0) :look-at-object? t)
  ;; (send *robot* :angle-vector   #f(0.497866 -11.5794 -10.9614 38.9877 -18.0864 11.1527 -0.660753 10.6076 -10.7331 38.745 -18.0875 -10.9056 0.0 3.0 0.065955 -0.009049 20.0 37.0 34.7493 -74.2251 -25.5205 -90.7107 6.29055 1.54029 30.4263 -37.0 36.0901 76.0089 25.7384 -90.2711 -3.90134 -2.16365 30.4717))
  (send *robot* :angle-vector #f(0.198761 -11.1901 -15.2719 36.987 -13.1589 10.9564 -0.398351 10.8076 -15.1034 36.9009 -13.2446 -10.8891 0.0 3.0 -0.026228 -0.005043 18.4018 37.0 38.4206 -88.7584 -30.2222 -80.624 -3.43998 -3.26935 26.4778 -37.0 39.2722 90.1039 30.227 -79.7785 4.40195 2.48036 26.5397))
  (send *robot* :head-neck-p :joint-angle 10)
  (send *robot* :move-coords (make-coords :pos #f(0 300 0)) (send *robot* :lleg :end-coords))
  (objects (list *robot* ))
  (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?  )  
  (setq zmp-vector (send *ri*  :zmp-vector))
  (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?)
  (if send?
      (progn 
        (if (not fix-contact?)
            (progn
              (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
              (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t))
  	  )
        (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?  )))
  )

(defun reaching-pose (&key (send? nil) (fix-contact? nil) (look-at-object? nil))
  (send *ri* :start-impedance :arms)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (my-init-pose :move-foot? nil :send? send?)
  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *robot* :angle-vector #f(-10.6248 -6.65663 -106.861 50.9002 3.98799 12.4148 10.6723 6.7806 -106.801 51.0553 3.75381 -12.5862 -0.183996 27.0 0.021731 0.021981 18.0 37.0 -5.52897 -49.7907 -25.152 -80.696 57.979 -29.8714 13.8606 -37.0 -5.22003 50.1071 25.3023 -80.8198 -57.8915 29.7632 13.9217))
  (send *robot* :move-coords (make-coords :pos #f(0 300 0)) (send *robot* :lleg :end-coords))
  (objects (list *robot* ))
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn 
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t)))
  ;;       (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?  )))
)

(defun heave-with-sensor
    (larm-move-offset rarm-move-offset times
                      &key  (fix-contact? t) (send? t) (wait? t) (anti-slip-rate 0.5) (check? nil)
                      (reset? t) (apply-external-force? t) (avoid-collision? t)
                      (rleg-pos-shift #f(0 0 0))
                      (lleg-pos-shift #f(0 0 0))
                      (look-at-object? t))
  
  (setq l-unit (* (/ 1.0 times) larm-move-offset))
  (setq r-unit (* (/ 1.0 times) rarm-move-offset))
  ;; (setq fc (make_plane))
  ;; (setq normal-vec (send (elt (send fc :get-face :prism :top) 0) :normal))
  ;; (setq external-coords-list (list (send *robot-env* :robot :get :rarm-contact-coords)
  ;;                                  (send *robot-env* :robot :get :larm-contact-coords)))
  (setq larm-pos-diff #f(0 0 0))
  (setq rarm-pos-diff #f(0 0 0))  
  (dotimes (i times)
    (setq fc (make_plane))
    (setq ret-larm (get-target-pose fc "larm"))
    (setq ret-rarm (get-target-pose fc "rarm"))
    (format t "ret-larm = ~A~%" ret-larm)
    (format t "ret-rarm = ~A~%" ret-rarm)
    (if (< 0 i)
        (progn
          (setq larm-pos-diff (v- (elt ret-larm 0) (elt ret-larm-save 0)))
          (setq rarm-pos-diff (v- (elt ret-rarm 0) (elt ret-rarm-save 0)))
          (format t "ret-larm-save = ~A~%" ret-larm-save)
          (format t "ret-larm-save = ~A~%" ret-larm-save)
          (format t "larm-pos-diff = ~A~%" larm-pos-diff)
          (format t "rarm-pos-diff = ~A~%" rarm-pos-diff)))
          
    (setq normal-vec (send (elt (send fc :get-face :prism :top) 0) :normal))
    (if (< (elt normal-vec 2) 0)
        (setq normal-vec (scale -1.0 normal-vec)))
    (format t "normal-vec =~A~%" normal-vec)
    (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate (v+ (scale anti-slip-rate larm-pos-diff) (scale l-unit normal-vec)) :world))
    (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (v+ (scale anti-slip-rate rarm-pos-diff) (scale r-unit normal-vec)) :world ))
    (format t "larm-coords = ~A~%" larm-coords)
    (format t "rarm-coords = ~A~%" rarm-coords)    
    (setq larm-force (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil))
    (setq rarm-force (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil))
    (setq external-force-list (list (make-array 6 :element-type float-vector :initial-contents larm-force :initial-element 0)
                                    (make-array 6 :element-type float-vector :initial-contents rarm-force :initial-element 0)))
    (setq ret (ik-opt rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :external-force-list external-force-list ))
    (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object? :wait? t)

    (if check?
        (progn
    (setq drop-flag (check-force-pcl :working? t :thre-list (list 10 10)))
    (cond
         ((or (= drop-flag 1) (= drop-flag 2))
          (progn
            (setq fc (make_plane))
            (if (= drop-flag 1)
                (progn
                  (setq ret-larm (get-target-pose fc "larm"))
                  (setq larm-coords (make-coords :pos (elt ret-larm 0) :rpy (elt (rpy-angle (elt ret-larm 1)) 0)))
                  (setq larm-coords (send larm-coords :rotate (deg2rad -20) :z)))
              (progn
                (setq ret-rarm (get-target-pose fc "rarm"))
                (setq rarm-coords (make-coords :pos (elt ret-rarm 0) :rpy (elt (rpy-angle (elt ret-rarm 1)) 0)))
                (setq rarm-coords (send rarm-coords :rotate (deg2rad 20) :z))))
            (ik-opt rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? 2 :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? t :rleg-pos-shift #f(0 0 0) :lleg-pos-shift #f(0 0 0))
            (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)))
         ((= drop-flag 0)
          (progn
            ))
         ((= drop-flag 3)
          (progn
            (format t "stop~%")
            )))
    ))
    (setq ret-larm-save ret-larm)
    (setq ret-rarm-save ret-rarm)
    )
  )
(defun send-pose-and-move-on-foot
    (&key  (fix-contact? t) (send? t)
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 0))
               (lleg-pos-shift #f(0 0 0))
               (look-at-object? t))
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
        (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?))))
;;1203
;;progn (load "demo.l ") (prepare-simulator)
;;reaching-pose :send? t
;;check-force-both-arms 
(defun check-force-both-arms
    (&key  (fix-contact? 2) (send? t) (cog #f(0 0 700))
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 0))
               (lleg-pos-shift #f(0 0 0))
               (look-at-object? nil)
               (thre -10.0))
  (if (not (= 0 (check-pcl)))
      (progn
        (format t "adjust~%")
        (move-hand :larm #f(100 30 0))
        (move-hand :larm #f(100 30 0))))
  
  (setq larm-coords-tmp (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rarm-coords-tmp (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq center-pos-tmp (scale 0.5 (v+ (send larm-coords-tmp :worldpos) (send rarm-coords-tmp :worldpos))))  
  (setq larm-force (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil))
  (setq rarm-force (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil))
  (setq cnt 0)
  (while (or (< thre (elt larm-force 2)) (< thre (elt rarm-force 2)))
    ;; (if (< (elt larm-force 2) 0) 
    (move-hand :larm #f(30 30 0))
    ;; (if (< (elt rarm-force 2) 0) 
    (move-hand :rarm #f(30 -30 0))
    (setq larm-force (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil))
    (setq rarm-force (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil))
    (setq cnt (+ cnt 1)))
  (format t "grasp~%")
  (setq total-force (v+ larm-force rarm-force))
  (setq lg (abs (elt larm-force 2)))
  (setq rg (abs (elt rarm-force 2)))
  (setq tg (abs (elt total-force 2)))
 
  (setq com-expected (v+ (scale (/ lg tg) (send larm-coords-tmp :worldpos)) (scale (/ rg tg) (send rarm-coords-tmp :worldpos))))
  (format t "larm-pos = ~A~%" (send larm-coords-tmp :worldpos))
  (format t "rarm-pos = ~A~%" (send rarm-coords-tmp :worldpos))  
  (format t "center-pos = ~A~%" center-pos-tmp)
  (format t "com-expected = ~A~%" com-expected)
  (setq dif (v- com-expected center-pos-tmp))
  (setq larm-target-coords (send larm-coords-tmp :translate dif :world))
  (setq rarm-target-coords (send rarm-coords-tmp :translate dif :world))
  (format t "dif = ~A~%" dif)  
  (format t "larm-target coords= ~A~%" larm-target-coords)
  (format t "rarm-target coords= ~A~%" rarm-target-coords)  
  (move-hand :larm (scale (- cnt 1) #f(-30 -30 0)))
  (move-hand :rarm (scale (- cnt 1) #f(-30 30 0)))
  (setq lleg-pos-shift (v+ lleg-pos-shift (float-vector 0 (* 0.5 (elt dif 1)) 0)))
  (setq rleg-pos-shift (v+ rleg-pos-shift (float-vector 0 (* 0.5 (elt dif 1)) 0)))  
  (ik-opt rarm-target-coords larm-target-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift ;; :external-force-list external-force-lis   
          )
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn 
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t)))
  ;;       (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?)))
  )
;; send *ri* :set-gait-generator-param :default-step-time 1.2
