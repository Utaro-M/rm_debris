;; (require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")
;; (require "package://control_tools/log-tools.l")
(require "package://control_tools/play-hrpsysfile.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

;; (require "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")

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

(defun send-pose
    (time &key (pre-pose (send *robot* :angle-vector)) (pre-lleg  (send *robot* :lleg :end-coords :copy-worldcoords) ) (force-move nil) (wait? t))
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
      (send *robot* :angle-vector pre-pose)
      (send *robot* :move-coords (send pre-lleg :translate #f(0 0 100)) (send *robot* :link "LLEG_LINK5"))
      (objects)
      (print "pre pose")
      )
    ))


(defun reach
    (&key (fix-contact? nil)
          (send? t)
          (wait? t)
          (avoid-collision? nil)
          (apply-external-force? nil)
          (rarm-coords (make-coords :pos (float-vector 390 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
          (larm-coords (make-coords :pos (float-vector 390  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90)))) )
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  ;; (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (send *robot* :reset-manip-pose)
  (send *coords* :set-rleg (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  ;; (send (car (send *robot* :links)) :newcoords (make-coords :pos #f(0 0 600) :rpy (float-vector 0 (deg2rad 50) 0)))

  ;;もどす
  ;; (send *robot* :move-coords (make-coords :pos #f(0 0 600) :rpy (float-vector 0 (deg2rad 50) 0)) (car (send *robot* :links)))
  
  
  ;; (setq rarm-coords (make-coords :pos (float-vector 290 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
  ;; (setq larm-coords (make-coords :pos (float-vector 290  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))
  
  ;; (setq rarm-coords (make-coords :pos (float-vector 390 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
  ;; (setq larm-coords (make-coords :pos (float-vector 390  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))

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
  ;; (ik-walk rarm-coords larm-coords :cog #f(0 0 1000) :move-root-link :z)  
  ;; (ik-st rarm-coords larm-coords :cog #f(0 0 1000) :move-root-link :z)
  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (ik-opt rarm-coords larm-coords :only-kinematics nil :avoid-collision nil :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait?)
  
  (if send?
      (progn 
        (if (not fix-contact?)
            (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords)   (send *robot* :lleg :end-coords :copy-worldcoords)))
;  (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog #f(0 0 1000))

        ;;もどす
        (send-pose movetime :force-move flag-simulator :pre-pose pre-pose :pre-lleg pre-lleg)))
  ;; (my-ik #f(0 0 40) #f(0 0 40))

  )

(defun start-coords()
  (send *robot* :move-coords (make-coords :pos #f(0 100 100) :rpy #f(0 0 0) ) (send *robot* :link "LLEG_LINK5"))
  (objects))

;;持ち上げる
(defun heave
    (rarm-move larm-move &key  (fix-contact? t) (send? t) (cog #f(0 0 700)) (degree 0) (wait? t) (reset? t) (apply-external-force? t))
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


  ;; (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)
  ;; (ik-walk rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)

  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (ik-opt rarm-coords larm-coords :only-kinematics nil :avoid-collision nil :optimize-torque? nil :fix-contact? fix-contact? :reset? reset? :apply-external-force? apply-external-force?)
  (if send?
      (progn
        (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg)
        (if (not fix-contact?)
            (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords)   (send *robot* :lleg :end-coords :copy-worldcoords) ;; :which? :rleg
                          ))
        ))
  
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



(defun change-arm
    (offset arm)

  ;; ;;jikki
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  
  ;; ;;腕を引く
  ;; (send *robot* arm :move-end-pos (scale offset #f(-1.5 0 0)) :local)
  ;; (send-pose movetime :force-move flag-simulator)

  ;; (send *ri* :start-grasp arm)

  ;; ;;腕を下へ
  ;; (if (eq arm :rarm)
  ;;     (send *robot* arm :move-end-pos (scale offset #f(0 2 0)) :local)
  ;;   (send *robot* arm :move-end-pos (scale offset #f(0 -2 0)) :local))
  ;; (send-pose movetime :force-move flag-simulator)

  ;; ;;腕を戻す
  ;; ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  ;; (if (eq arm :rarm)
  ;;     (send *robot* arm :move-end-pos (scale offset #f(2 -0.5 0)) :local)
  ;;   (send *robot* arm :move-end-pos (scale offset #f(2 -0.5 0)) :local))
  ;; (send-pose movetime :force-move flag-simulator)
  
  ;;choreonoid
  (send *robot* :angle-vector (send *ri* :state :angle-vector))  
  ;;腕を引く
  (send *robot* arm :move-end-pos (scale offset #f(-1 0 0)) :local)
  (send-pose movetime :force-move flag-simulator)

  (send *ri* :start-grasp arm)  
  ;;腕を下へ
  (if (eq arm :rarm)
      (send *robot* arm :move-end-pos (scale offset #f(0 1.5 0)) :local)
    (send *robot* arm :move-end-pos (scale offset #f(0 -1.5 0)) :local))
  (send-pose movetime :force-move flag-simulator)
  ;;腕を戻す    
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

(defun my-init-pose
    (&key (move-foot? nil) (only-upper? nil))
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (objects)
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
  (send-pose movetime :force-move flag-simulator)
  ;; (send *ri* :hand-reset) ;;実機では必要？choreonoidでは不要？
  (format t "left or right ?  ~A" (elt (send (car (send *robot* :torso)) :worldpos) 1))
  (if move-foot?
      (if (< 0 (elt (send (car (send *robot* :torso)) :worldpos) 1))
          (progn
            (start-coords)
            (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) :which? :lleg))
        ;;     (progn
        ;;       (start-coords)
        ;;       (move-on-foot rleg-coords lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords)))
        ;;   (progn
        ;;       (start-coords)
        ;;       (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords)))
        ;;   )
        (progn
          (start-coords)
          (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) :which? :rleg))
        )
    )
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




;;progn (send *ri* :set-log-maxlength 30)(unix:sleep 2) (send *ri* :start-log)(heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (unix:sleep 2)  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (save-log :fname "check")

;progn  (send *ri* :start-log) (unix:sleep 2) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (unix:sleep 2)  (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (heave #f(0 -200 0) #f(0 200 0) :cog #f(100 0 700)) (check-while-working) (save-log :fname "check")


;;(save-log :fname "rm_board" :log-dir "matsuura/log")
;heave #f(0 -200 0) #f(0 200 0) :move-on-foot? nil :fix-contact? t
