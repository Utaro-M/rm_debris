(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")
(load "package://control_tools/log-tools.l")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )


(objects (list *robot*))
(setq flag-simulator nil) ;;simulatorではｔ
(setq movetime 16000)

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
(defun reach()
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :reset-manip-pose)
  ;; (send (car (send *robot* :links)) :newcoords (make-coords :pos #f(0 0 600) :rpy (float-vector 0 (deg2rad 50) 0)))
  (send *robot* :move-coords (make-coords :pos #f(0 0 600) :rpy (float-vector 0 (deg2rad 50) 0)) (car (send *robot* :links)))
  ;; (setq rarm-coords (make-coords :pos (float-vector 550 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  -90))))
  ;; (setq larm-coords (make-coords :pos (float-vector 550  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad   90))))
  (setq rarm-coords (make-coords :pos (float-vector 420 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 420  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))  
  (setq rleg-coords (make-coords :pos #f(0 -100 0))
	lleg-coords (make-coords :pos #f(0  100 0)))
  (ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords)
  (send-pose movetime :force-move flag-simulator)
  ;; (ik #f(0 0 40) #f(0 0 40))

  )



(defun ik
    (rarm-coords larm-coords &key rleg lleg (cog #f(0 0 700) ) (move-root-link t))
  ;; (let* ((rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  ;;        (lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  ;;        ;; (rarm-coords (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm ) :rotate (deg2rad 90) :z)) 
  ;;        ;; (larm-coords (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm ) :rotate (deg2rad 90) :z)))

  ;;        ;; (rarm-coords (send *robot* :rarm :end-coords :worldpos (make-coords :pos rarm :rpy (float-vector 0 0 (deg2rad 90)))))
  ;;        ;; (larm-coords (send *robot* :rarm :end-coords :worldpos (make-coords :pos rarm :rpy (float-vector 0 0 (deg2rad 90)))))         
  ;;        ;; (larm-coords (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm ) :rotate (deg2rad 90) :z)))
  ;;        )

    (send *robot* :torso :waist-p :joint-angle 10)
    
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
          ;; :root-link-virtual-joint-weight #f(0.1 0.1 0.1  1 1 1)
	  :root-link-virtual-joint-weight #f(0.1 0.1 0.1  100 100 100)

          :translation-axis (list t t t t)
          :rotation-axis (list t t t t)
          :target-centroid-pos (v+ (midpoint 0.5 (send rleg :worldpos) (send lleg :worldpos))
                                   cog)
          ;; :debug-view nil
          :revert-if-fail nil
          :cog-translation-axis move-root-link
          )

    (send rarm-coords :draw-on :flush nil :size 300)
    (send larm-coords :draw-on :flush t   :size 300)
    )

(defun send-pose (time &key (force-move nil))
  (objects)
  (when (or force-move (y-or-n-p "Do you move real machine?"))
    (send *ri* :angle-vector (send *robot* :angle-vector) time)
    (send *ri* :wait-interpolation)
    )
  )

(defun demo ()
  ;; (reach #f(360 0 -700) #f(360 0 -700))
  ;; (reach #f(260 0 -700) #f(260 0 -700))
  ;; (reach #f(200 0 -710) #f(200 0 -710))
  (init-pose)
  ;; (send *ri* :set-impedance-controller-param :arms :D-p 400 :K-r 1e5 :D-r 1e5)
  ;; (send *ri* :start-impedance :arms :K-r 140.0 :D-r 200.0 :M-r 5.0 :K-p 300.0 :D-p 600.0 :M-p 10.0)
  
  (reach)

  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0  300 0)))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 -300 0)))
  ;; ;; (ik #f(0 -280 0) #f( 0 280 0))
  ;; (ik rarm-coords larm-coords)
  ;; ;; (send-pose 3000 :force-move t)
  ;; (send-pose 3000 :force-move nil)

  ;; (heave #f(40  -400 0) #f(40  400 0))
  ;; (heave #f(40  -300 0) #f(40  300 0))
  (heave #f(60  -700 0) #f(60  700 0) :degree 10)
  (setq flag-simulator nil) ;;simulatorではｔ

  ;; (heave #f(10 0 0) #f(10 0 0) :cog #f(0 0 900) )
  ;; (heave #f(40  -100 0) #f(40  100 0))

  ;; (send *ri* :start-impedance :arms :K-r 140.0 :D-r 200.0 :M-r 5.0 :K-p 340.0 :D-p 600.0 :M-p 10.0)

  ;; (send *ri* :go-pos 0.3 0 0)
  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(40  200 0)))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(40 -200 0)))
  ;; (ik rarm-coords larm-coords)
  ;; ;; (ik #f(0 -100 0) #f( 0 100 0))
  ;; ;; (send-pose 3000 :force-move t)
  ;; (send-pose 3000 :force-move nil)
  ;; (heave #f(40  200 0) #f(40  -200 0))
  
  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0  100 0)))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 -100 0)))
  ;; ;; (ik #f(0 -280 60) #f( 0 280 60))
  ;; ;; (send-pose 3000 :force-move t)
  ;; (send-pose 3000 :force-move nil)
  ;; (heave #f(0  100 0) #f(0  -100 0))
  
  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(0  100 0)))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(0 -100 0)))
  ;; (send-pose 3000 :force-move nil)

  ;; (ik #f(0 -100 0) #f( 0 100 0))
  ;; (send-pose 3000 :force-move t)  

  )

(defun heave
    (rarm-move larm-move &key (cog #f(0 0 700)) (degree 0))
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (setq rarm-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm-move))
  ;; (setq larm-coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move))
  (setq rarm-coords (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm-move) :rotate (deg2rad degree) :z))
  (setq larm-coords (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move) :rotate (* -1 (deg2rad degree)) :z))

  ;; (setq rleg-coords (make-coords :pos #f(0 -100 0))
  ;;       lleg-coords (make-coords :pos #f(0  100 0)))
  (setq rleg-coords  (send *robot* :rleg :end-coords :copy-worldcoords)
        lleg-coords  (send *robot* :lleg :end-coords :copy-worldcoords))
  
  (ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog)
  (send-pose movetime :force-move flag-simulator)
  )

;;手先位置は固定で腰を高くするheight 1000とか
(defun stand-up
    (height &key (cog #f(0 0 700)) (degree 0) (move-root-link :z))
  
  (setq rarm-coords-const (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq larm-coords-const (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rleg-coords  (send *robot* :rleg :end-coords :copy-worldcoords)
        lleg-coords  (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-coords (make-coords :pos (float-vector 0 0 height) :rpy (float-vector 0 (deg2rad 50) 0)) (car (send *robot* :links)))
  
  (ik rarm-coords-const larm-coords-const :rleg rleg-coords :lleg lleg-coords :cog cog :move-root-link move-root-link)
  (send-pose movetime :force-move flag-simulator)
  )

(defun push-board ()
  (if auto-detect-p
      (send *ri* :set-object-turnaround-ref-force :max-ref-force (* 9.8 6.0))
    (progn
      (send *ri* :set-ref-force (float-vector 0 0 (* 9.8 2.5 -1)) 2000 :arms)
      (send *ri* :wait-interpolation-seq))))
  
 
;;   (ik rarm-coords larm-coords)
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
  (init-pose)
  (unix:sleep 2)
  (reach)
  (unix:sleep 2)
  
  (save-log :fname name)
  )

(defun init-pose()
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (objects)
  (send *robot* :reset-manip-pose)
  (send-pose 3000 :force-move flag-simulator)
  )



(print "init-pose")
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
