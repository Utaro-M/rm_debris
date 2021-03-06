(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")


(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )
(objects (list *robot*))


;;init
;;reach で棒にてを伸ばす


;; ;;plot
;; (send *ri* :set-log-maxlength (* 30 500))
;; (load "package://control_tools/play-hrpsysfile.l")

;; (setq target (instance arrow-object :init))
;; (send target :newcoords
;;         (make-coords :pos (float-vector 0 0 0) :rpy (list 0 0 0)))
;; (send *robot* :larm :inverse-kinematics target  :move-target (send *robot* :rarm-end-coords) :rotation-axis :z );;:debug-view t)

;; send *robot* :rarm :move-end-pos #f(100 0 0)

(defun grab_on_table ()
(setq *rarm-end* (send *robot* :rarm :end-coords :copy-worldcoords))
;; (send *rarm-end* :translate (float-vector 140.0 0.0 60.0))
;; (send *rarm-end* :rotate (/ pi 4) :y)
(send *rarm-end* :translate (float-vector 200.0 0.0 20.0))
(send *rarm-end* :rotate (/ pi 2) :y)
(send *robot* :rarm :inverse-kinematics *rarm-end* :rotation-axis :z)

;; (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
;; (send *ri* :wait-interpolation)
(send-angle-vector 3000 :force-move t)
(send *robot* :rarm :move-end-pos #f(140 0 0))
;; (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
;; (send *ri* :wait-interpolation)
(send-angle-vector 3000 :force-move t)

(send *ri* :start-grasp)

(snap 0 (/ -pi 2) 0)

(objects (list *robot* (send *robot* :rarm :end-coords :copy-worldcoords)))
)


(defun reach()
  (setq *rarm-end* (send *robot* :rarm :end-coords :copy-worldcoords))
  (send *rarm-end* :translate (float-vector 210.0 80.0 0.0))
  (send *rarm-end* :rotate (/ pi 2) :y)
  (send *robot* :rarm :inverse-kinematics *rarm-end* :rotation-axis :z)
  (send-angle-vector 3000 :force-move t)
  (send *robot* :rarm :move-end-pos #f(140 0 0))
  (send-angle-vector 3000 :force-move t)
  (send *robot* :rarm :move-end-pos #f(0 0 20))   
  (send-angle-vector 3000 :force-move t)
  )


;;実機でも使える関数

;;reset-pose
(defun init ()
  (send *robot* :reset-manip-pose)
;;  (send *ri* :start-impedance :rarm :d-r 5 :k-r 20) 
  (send-angle-vector 3000 :force-move t)
;;  (send *ri* :hand-reset) ;;実機では必要？choreonoidでは不要？
    )

;;棒をつかむ
(defun grab ()
  (send *ri* :start-grasp :rarm)
  )

;;手を開く
(defun open ()
  (send *ri* :stop-grasp :rarm)
  )

;;手先を平行移動
(defun shake (x y z)
  (send *robot* :rarm :move-end-pos (float-vector x y z))
  (send-angle-vector 3000 :force-move t)
  )

;;棒をふる(手首を回転）
(defun snap (x y z)
  (setq *rarm-end* (send *robot* :rarm :end-coords :copy-worldcoords))

  (if x
      (send *rarm-end* :rotate (deg2rad x) :x)
    )
  (if y
      (send *rarm-end* :rotate (deg2rad y) :y)
    )
  (if z
      (send *rarm-end* :rotate (deg2rad z) :z)
    )
  
  (send *robot* :rarm :inverse-kinematics *rarm-end* :rotation-axis :z)
  (send-angle-vector 3000 :force-move t)
  )

(defun send-angle-vector (time &key (force-move nil))
  (objects)
  (when (or force-move (y-or-n-p "Do you move real machine?"))
    (send *ri* :angle-vector (send *robot* :angle-vector) time)
    (send *ri* :wait-interpolation)
    )
  )

(defun demo ()
  (snap 0 (/ (* 2 pi) 3) 0)
  (snap 0 (/ (* -2 pi) 3) 0)
  ;; (snap (/ pi 3) 0 0)
  ;; (snap (/ -pi 3) 0 0)  
    )

;;logを取る
;;0908いろんな長さで叩いてみる実験用
;;log "contact_70_7~~~" 7　とか
(defun log (name ang)
  (progn (send *ri* :start-log) (unix:sleep 3) (snap 0 ang 0) (unix:sleep 2) (snap 0 (- 0 ang) 0) (unix:sleep 2) )
  (save-log :fname name)
  )

(defun grab-and-release (name)
  (progn (send *ri* :start-log) (unix:sleep 3) (grab) (unix:sleep 2) (open) (unix:sleep 2) )
  (save-log :fname name)
  )

;;0914 predict.lにも同じものがある
;; #!/usr/bin/env roseus
;; (ros::load-ros-manifest "roseus")
;; (ros::roseus "listener")
;; ;; method call
;; (defclass string-cb-class
;;   :super propertied-object
;;   :slots ())
;; (defmethod string-cb-class
;;   (:init () (ros::subscribe "off_rhsensor" geometry_msgs::WrenchStamped  #'send self :string-cb))
;;   (:string-cb (msg) (print (list 'cb self (send msg :force)))))
;; (setq m (instance string-cb-class :init))



(defun display-arrow ()
  
  (objects (list *robot*))
  (setq *viewer* (send *irtviewer* :viewer))
  (setq idx 0)
  (do-until-key
   (unix:sleep 1)
   (print "arrow")
   ;; (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
   (send (send *robot* :rarm :end-coords ) :draw-on :flush nil :size 300)
   (setq endpos (send *robot* :rarm :end-coords :worldpos)
         ;; force-vec (scale 10 (send *ri* :state :off-force-vector :rarm)))
         force-vec (scale 10 (send *ri* :state :off-force-vector :rarm)))
   ;;force-vec (scale (* 10 (expt -1 idx)) (send *ri* :state :off-force-vector :rarm)))
   (send *viewer* :draw-arrow endpos (v+ endpos force-vec))
   (print  force-vec)
   (print (v+ endpos force-vec))
   (if (> (elt (v+ endpos force-vec) 0) 300)
       (break))
   ;; (print (scale 100 (send *ri* :state :potentio-vector )))
   ;;     (send (send *irtviewer* :viewer)
   ;; (send *irtviewer* :draw-objects :flush t)
   ;; (send (send *irtviewer* :viewer) :flush)
   (send *viewer* :flush)
   (send *viewer* :draw-objects :flush nil)
   (x::window-main-one)
   (incf idx)
   )
  )

(defun series()
  (init)
  (grab)
  
  (send *ri* :start-log )
  ;;(snap 0 (/ (* 1 pi) 3) 0)
  (unix:usleep 3000)
  (save-log :fname "stick_contact_i")
  (unix:usleep 1000)
  
  (send *ri* :start-log )
  (snap 0 (/ (* 1 pi) 2) 0)
  (unix:usleep 3000)
  (save-log :fname "stick_contact_f")
  (unix:usleep 1000)
  
  (snap 0 (/ (* -1 pi) 2) 0)

  (send *ri* :start-log )
  (snap (/ (* -1 pi) 2) 0  0)
  (unix:usleep 3000)
  (save-log :fname "stick_contact_l")
  (unix:usleep 1000)

  (snap (/ (* 1 pi) 2) 0  0)  

  (send *ri* :start-log )
  (snap (/ (* 1 pi) 2) 0  0)
  (unix:usleep 3000)
  (save-log :fname "stick_contact_r")
  (unix:usleep 1000)    
  
  )

(print "reset-pose init ")
(print "grab stick grab")
(print "translate end-effecter shake(x y z)")
(print "snap hand snap(x y z) ")
(print "save log log ")


;;(save-log :fname "stick


