;; (require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")
;; (require "package://control_tools/log-tools.l")
(require "package://control_tools/play-hrpsysfile.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

(require "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")

(require "./class.l")
(load "./my-ik.l")
(require "../../euslisp/save-log.l")
(require "~/research/rm_debris/src/predict_from_multi_pose.l")


(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
 ) 

(objects (list *robot*))
(unless (boundp 'flag-simulator)
  (setq flag-simulator nil) ;;simulatorではｔ
  (setq movetime 16000))
;;(send *ri* :save-log "~/matsuura/log/testes" :make-directory t)

(setq *coords* (instance my-coords :init
                       (copy-object (send *robot* :rarm :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :larm :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :rleg :end-coords :copy-worldcoords))
                       (copy-object (send *robot* :lleg :end-coords :copy-worldcoords))
                       #f(0 0 700)
                       ;; (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
                       ;; (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
                       ))

(setq *poses* (instance poses :init
                       (send *robot* :angle-vector)
                       (send *ri* :state :angle-vector)))

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


;;(float-vector 0 0 (elt (send (car (send *robot* :links)) :worldpos) 2))

(defun send-pose (time &key (force-move nil))

  (objects)
  (when (or force-move (y-or-n-p "Do you move real machine?"))
    (send *ri* :angle-vector (send *robot* :angle-vector) time)
    (if wait?
        (progn
          (print "wait-interpolation")
          (send *ri* :wait-interpolation)))
    )
  )
(defun reach()
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  ;; (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
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
  (my-ik rarm-coords larm-coords :cog #f(0 0 1000) :move-root-link :z)
  ;; (ik-walk rarm-coords larm-coords :cog #f(0 0 1000) :move-root-link :z)  
  ;; (ik-st rarm-coords larm-coords :cog #f(0 0 1000) :move-root-link :z)  

;  (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog #f(0 0 1000))

  ;;もどす
  (send-pose movetime :force-move flag-simulator)
  ;; (my-ik #f(0 0 40) #f(0 0 40))

  )

;;持ち上げる
(defun heave
    (rarm-move larm-move &key (cog #f(0 0 700)) (degree 0) (wait? t))
  ;; (send *robot* :move-coords (make-coords :pos #f(0 0 600) :rpy (float-vector 0 (deg2rad 50) 0)) (car (send *robot* :links)))
  (send *robot* :fix-leg-to-coords (make-coords))
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


  (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)
  ;; (ik-walk rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)
  (send-pose movetime :force-move flag-simulator :wait? wait?)
  ;; (setq rarm-ref-f (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  ;; (setq larm-ref-f (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  ;; (check-while-still rarm-ref-f larm-ref-f )

  ;;kokokara
  ;;masteremergencyではmy-kiのみを使っている、ik-walkの方だとik-walkをikとして使うことになる
  ;;conflictしたのでとりあえず全部コメントアウト
  ;;;;load した状態でmy-ikを使うとセグフォ
  ;;;; (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)  
  ;;(ik-walk rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)

;;  (send *coords* :set-rleg (send *robot* :rleg :end-coords :copy-worldcoords))
  ;;(send *coords* :set-lleg (send *robot* :rleg :end-coords :copy-worldcoords))
  ;;;; (ik-walk rarm-coords larm-coords :cog cog)
  ;;(move-on-foot rleg-coords (send *robot* :lleg :end-coords :copy-worldcoords)   (send *robot* :rleg :end-coords :copy-worldcoords))

  ;;(send-pose movetime :force-move flag-simulator)
  ;;(setq rarm-ref-f (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
  ;;(setq larm-ref-f (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
  ;;; (check-while-still rarm-ref-f larm-ref-f )


;; kokomade

  ;; (check rarm-coords larm-coords (list 80 80) 5)

  )


;; (setq waiting nil)

;;progn (load "heave.l ") (prepare-simulator)
;;progn (my-init-pose)   (send *robot* :fix-leg-to-coords (make-coords))
;;reach
;;progn   (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working)
;;これを繰り返すして200ずつ上げていくと途中で腕が外れる
;;progn  (send *ri* :start-log) (unix:sleep 2) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (unix:sleep 2)  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (unix:sleep 2)  (send *ri* :save-log "/tmp/check-while-working")
(defun check-while-working
    (&key (checking-time 6) (sleep-time 20000) (thre-list (list 22 22)) (fin 3))
  (format t "~%~%check while working~%")
  (setq initial (send *robot* :angle-vector)
        rarm-f-pre  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil)
        larm-f-pre  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil)
        angle-pre (send *ri* :state :angle-vector))

  
  (dotimes (i (/ (* checking-time 1000000) sleep-time))
    (setq rarm-f  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil))
    (setq larm-f  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil))
    ;;絶対値バージョン
    ;; (setq rarm-f-dif (abs (elt (v- rarm-f rarm-f-pre) 2)))
    ;; (setq larm-f-dif (abs (elt (v- larm-f larm-f-pre) 2)))
    (setq rarm-f-dif (elt (v- rarm-f rarm-f-pre) 2))
    (setq larm-f-dif (elt (v- larm-f larm-f-pre) 2))

    (setq drop (list nil nil))
    (format t "rarm-f-dif       = ~A~%" rarm-f-dif)
    (format t "larm-f-dif       = ~A~%" larm-f-dif)


    ;;drop判定
    (if (>  rarm-f-dif (* +1 (elt thre-list 0)))
        (setq drop (list-insert t 0 drop)))

    (if (>  larm-f-dif (* +1 (elt thre-list 1)))
        (setq drop (list-insert t 1 drop)))

    (if (or (elt drop 0) (elt drop 1))
        (progn
          (print "drop")
          (send *ri* :stop-motion)
          (print "stop motion")
          (if (and (elt drop 0) (elt drop 1))
              (progn
                (print "drop both arm")
                (setq flag 0)
                (return 0))
            (progn
              (cond ((elt drop 0)
                     (progn
                       (print "drop rarm")
                       (setq flag 1)
                       (return 1)))
                    ((elt drop 1)
                     (progn
                       (print "drop larm")
                       (setq flag 2)
                       (return 2))))))))
            ;; (progn
            ;;   (if (< larm-f-dif rarm-f-dif)
            ;;       (progn
            ;;         (print "drop rarm")
            ;;         (setq flag 1)
            ;;         (return 1))
            ;;     (progn
            ;;       (print "drop larm")
            ;;       (setq flag 2)
            ;;       (return 2))))
            ;; )))

    ;;動作終了判定
    (setq tmp (send *ri* :reference-vector))
    (setq angle-vector-dif (abs (norm (v- initial tmp))))
    (format t "angle-vector-dif = ~A~%" angle-vector-dif)
    (if (< angle-vector-dif 0.5)
        (progn
          (print "goal")
          (setq flag 10)
          (return angle-vector-dif))
      )
    
    (setq angle-pre (send *ri* :state :angle-vector))
    (setq rarm-f-pre rarm-f)
    (setq larm-f-pre larm-f)
    (unix:usleep sleep-time)
    )
  (format t "flag = ~A~%" flag)
  flag
  ;; (if (or (elt drop 0) (elt drop 1))
  ;;     (if (and (elt drop 0) (elt drop 1))
  ;;         t
  ;;       (if (< larm-f-dif rarm-f-dif)
  ;;           1)
  ;;       2)
  ;;   (progn
  ;;     (print "time over")
  ;;     nil))
  )


  
;;静止時の異常検知
;;前の動作終了時の力をref-fとして与えておｓ子から外れると落としたと判断
(defun check-while-still
    (rarm-ref-f larm-ref-f &key (checking-time 6) (sleep-time 100000) (thre-list (list 10 10)) (fin 3))

  (format t "~%~%check while still~%")
  (setq angle-pre (send *ri* :state :angle-vector))

  (setq flag 10)
  (dotimes (i (/ (* checking-time 1000000) sleep-time))
    (setq rarm-f  (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
    (setq larm-f  (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))
    ;; (setq rarm-f-dif (norm (v- rarm-ref-f rarm-f) ))
    ;; (setq larm-f-dif (norm (v- larm-ref-f larm-f) ))
    ;;絶対値バージョン
    ;; (setq rarm-f-dif (abs (elt (v- rarm-ref-f rarm-f) 2)))
    ;; (setq larm-f-dif (abs (elt (v- larm-ref-f larm-f) 2)))
    (setq rarm-f-dif (elt (v- rarm-ref-f rarm-f) 2))
    (setq larm-f-dif (elt (v- larm-ref-f larm-f) 2))
      
    (setq drop (list nil nil))
    (format t "rarm-f-dif      = ~A~%" rarm-f-dif)
    (format t "larm-f-dif      = ~A~%" larm-f-dif)

    ;;drop判定
    (if (> rarm-f-dif (* +1 (elt thre-list 0)))
        (setq drop (list-insert t 0 drop)))
    (if (>  larm-f-dif (* +1 (elt thre-list 1)))
        (setq drop (list-insert t 1 drop)))
    
    (if (or (elt drop 0) (elt drop 1))  
        (progn
          (print "drop")
          (send *ri* :stop-motion)
          (print "stop motion")
          (if (and (elt drop 0) (elt drop 1))
              (progn
                (print "drop both arm")
                (setq flag 0)
                (return 0))
            (progn
              (cond ((elt drop 0)
                     (progn
                       (print "drop rarm")
                       (setq flag 1)
                       (return 1)))
                    ((elt drop 1)
                     (progn
                       (print "drop larm")
                       (setq flag 2)
                       (return 2))))))))

    
    ;; (if (and (< rarm-f-dif fin) (< larm-f-dif fin))
    ;;     (progn
    ;;       (print "goal")
    ;;       (return 10)))

    (setq angle-pre (send *ri* :state :angle-vector))
    (unix:usleep sleep-time)
    )
  (format t "flag = ~A~%" flag)
  flag

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

(defun change-arm
    (offset arm)

  
  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  
  ;;腕を引く
  ;;jikki
  (send *robot* arm :move-end-pos (scale offset #f(-1.5 0 0)) :local)
  ;; ;;choreonoid
  ;; (send *robot* arm :move-end-pos (scale offset #f(-1 0 0)) :local)

  (send-pose movetime :force-move flag-simulator)
  (send *ri* :start-grasp arm)

  ;;腕を下へ
  ;;jikki
  (if (eq arm :rarm)
      (send *robot* arm :move-end-pos (scale offset #f(0 2 0)) :local)
    (send *robot* arm :move-end-pos (scale offset #f(0 -2 0)) :local))
  ;;choreonoid
  ;; (if (eq arm :rarm)
  ;;     (send *robot* arm :move-end-pos (scale offset #f(0 1.5 0)) :local)
  ;;   (send *robot* arm :move-end-pos (scale offset #f(0 -1.5 0)) :local))
  
  (send-pose movetime :force-move flag-simulator)

  ;;腕を戻す
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (if (eq arm :rarm)
      (send *robot* arm :move-end-pos (scale offset #f(2 -0.5 0)) :local)
    (send *robot* arm :move-end-pos (scale offset #f(2 -0.5 0)) :local))

  ;;choreonoid
  ;; (if (eq arm :rarm)
  ;;     (send *robot* arm :move-end-pos (scale offset #f(1.5 0 0)) :local)
  ;;   (send *robot* arm :move-end-pos (scale offset #f(1.5 0 0)) :local))
  
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
  (print "init pose ")
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :reset-manip-pose)
  ;; (send *ri* :hand-reset) ;;実機では必要？choreonoidでは不要？  
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

  ;; (progn (setq theta 80) (send *ri* :set-ref-force (scale 2 (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta))))) 2000 :larm) (change-arm 140 :rarm))
  ;; (progn (setq theta 80) (send *ri* :set-ref-force (scale 2 (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta))))) 2000 :rarm) (change-arm 140 :larm))

  ;; (progn (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms))

  ;; (send *ri* :go-pos 0.3 0 0)
  ;; (send *ri* :go-pos 0.3 0 0)

  ;; (setq theta 80) (send *ri* :set-ref-force (float-vector  (* (* -4 9.8) (* (cos (deg2rad theta)) (sin (deg2rad theta)))) 0  (* (* -4 9.8) (cos (deg2rad theta)))) 2000 :arms)
  
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


;;progn (send *ri* :set-log-maxlength 30)(unix:sleep 2) (send *ri* :start-log)(heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (unix:sleep 2)  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (save-log :fname "check")

;progn  (send *ri* :start-log) (unix:sleep 2) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (unix:sleep 2)  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (save-log :fname "check")


;;(save-log :fname "rm_board" :log-dir "matsuura/log")
