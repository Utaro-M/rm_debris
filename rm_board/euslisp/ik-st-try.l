(load "/home/utaro/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

(setq rarm-coords (make-coords :pos (float-vector 290 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
(setq larm-coords (make-coords :pos (float-vector 290  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))


(objects (list *robot*))

(defun tes
    (rarm-coords larm-coords
                 &key rleg lleg cog (move-root-link :z) (virtual-joint-weight #f(0.5 0.5 0.5  1 1 1))
                 
                 (optimize-torque? t)
                 (pre-process-func)
                 (post-process-func)
                 (stop 50))

  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  
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
          (list
           ;; (send (send *robot* :rleg :end-coords) :copy-worldcoords)
           ;; (send (send *robot* :lleg :end-coords) :copy-worldcoords)
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
   ;; (objects (list *robot* *ground* *ground2*)   )
)
