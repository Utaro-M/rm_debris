(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")


(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )
(objects (list *robot*))


(defun reach()
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq rarm-coords (make-coords :pos (float-vector 600 -400 400) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))
  (setq larm-coords (make-coords :pos (float-vector 600  400 400) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
  (ik rarm-coords larm-coords)
  ;; (send *robot* :rarm :wrist-y :joint-angle 30)
  ;; (send *robot* :larm :wrist-y :joint-angle -30)
  (send-pose 4600 :force-move t)
  ;; (ik #f(0 0 40) #f(0 0 40))
  ;; (objects (list *robot*))
  )



(defun ik (rarm-coords larm-coords)
  (let* ((rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
         (lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
         ;; (rarm-coords (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm ) :rotate (deg2rad 90) :z)) 
         ;; (larm-coords (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm ) :rotate (deg2rad 90) :z)))

         ;; (rarm-coords (send *robot* :rarm :end-coords :worldpos (make-coords :pos rarm :rpy (float-vector 0 0 (deg2rad 90)))))
         ;; (larm-coords (send *robot* :rarm :end-coords :worldpos (make-coords :pos rarm :rpy (float-vector 0 0 (deg2rad 90)))))         
         ;; (larm-coords (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm ) :rotate (deg2rad 90) :z)))
         )

    (send *robot* :torso :waist-p :joint-angle 10)
    
    (send *robot* :fullbody-inverse-kinematics
          (list rleg-coords
                lleg-coords
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
          :root-link-virtual-joint-weight #f(0.1 0.1 0.1  0.05 0.05 0.1)

          :translation-axis (list t t t t)
          :rotation-axis (list t t t t)
          :target-centroid-pos (v+ (midpoint 0.5
                                         (send *robot* :rleg :end-coords :worldpos)
                                         (send *robot* :lleg :end-coords :worldpos))
                                   #f(0 0 700))
          ;; :debug-view nil
          :revert-if-fail nil
          :cog-translation-axis t)

    (send rarm-coords :draw-on :flush nil :size 300)
    (send larm-coords :draw-on :flush t   :size 300)

    )
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
  (reach)

  ;; (setq rarm-coords (make-coords :pos (float-vector 600 -400 400) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))
  ;; (setq larm-coords (make-coords :pos (float-vector 600  400 400) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))

  (ik #f(0 -280 0) #f( 0 280 0))
  (send-pose 3000 :force-move t)

  (ik #f(0 -100 0) #f( 0 100 0))
  (send-pose 3000 :force-move t)  

  (ik #f(0 -280 60) #f( 0 280 60))
  (send-pose 3000 :force-move t)

  (ik #f(0 -100 0) #f( 0 100 0))
  (send-pose 3000 :force-move t)  

  )
(print "(reach #f(260 0 -700) #f(260 0 -700))")

(defun prepare-ex ()
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st))

  )

(defun init-pose()
  (send *robot* :reset-manip-pose)
  (send-pose 3000 :force-move t)
  )


(prepare-ex)
