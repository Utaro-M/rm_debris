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


(defclass point-num-save
  :super propertied-object
  :slots (larm-points-num rarm-points-num)
  )
(defmethod point-num-save 
  (:init (larm-points-num-init rarm-points-num-init)
         (send self :set-larm-points-num larm-points-num-init)
         (send self :set-rarm-points-num rarm-points-num-init)
         self
         )
  (:set-larm-points-num (val)
                        (setq larm-points-num val))
  (:set-rarm-points-num (val)
                    (setq rarm-points-num val))      
  )
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
