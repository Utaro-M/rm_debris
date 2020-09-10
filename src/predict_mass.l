;;質量の推定
;;git checkout 
;;export ROBOT=JAXON_RED
;;roseus 
(load "package://control_tools/log-tools.l")

;; (unless (and (boundp '*robot*) (boundp '*rs-list*))
(unless (boundp '*rs-list*)  
  (prepare-plot)
  (setq rs (car *rs-list*))
  )


(defun predict-mass
    (&key (free 2) (hold 10) (axis :x) (rs-list *rs-list*) (limbs (get-limbs-in-force-sensor-order *robot*)))
  (let* ((state-data
          (mapcar #'(lambda (l)
                      (mapcar #'(lambda (x) (send x :get (read-from-string (format nil "~A-off-force-vector" l)))) rs-list))
                  (list :rarm)))
    (state-time-data
      (mapcar #'(lambda (x) (send x :time )) rs-list)))
    ;;    (print (car state-data)
    (setq free-start-index (position-if #'(lambda (x) (and (< (- free 0.1) x) (< x (+ free 0.1) ))) state-time-data ))
    (setq free-end-index (position-if #'(lambda (x) (and (< (- (+ free 1) 0.1) x) (< x (+ (+ free 1) 0.1) ))) state-time-data ))
    
    (setq hold-start-index (position-if #'(lambda (x) (and (< (- hold 0.1) x) (< x (+ hold 0.1) ))) state-time-data ))
    (setq hold-end-index (position-if #'(lambda (x) (and (< (- (+ hold 1) 0.1) x) (< x (+ (+ hold 1) 0.1) ))) state-time-data ))

    

    (print free-start-index)    
    (print free-end-index)
    (print (list-length state-data))

    )
  )

;;(predict-mass)


  
