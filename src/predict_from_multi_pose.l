
;;predict.lのときは１姿勢を使って推定しようとしていたが、それでは歪対称行列の行列式が常に０であるため、求められないことがわかった
;; このコードでは複数姿勢の力モーメントを回転させて重ねて、拘束過多の疑似逆行列を用いて推定を行う


(load "./predict.l")

(defun pred-multi-test
    (force1 moment1 force2 moment2 &key (g  9.8))

  ;;make f-matrix
  (setq f force1)
  (setq f-matrix1 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))  
  (setq f force2)
  (setq f-matrix2 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))  
  (print (format nil "f-matrix1 = ~A" f-matrix1))

  ;;f-matrixをくっつける
  (setq f-matrix (matrix (matrix-row f-matrix1 0) (matrix-row f-matrix1 1)  (matrix-row f-matrix1 2) (matrix-row f-matrix2 0) (matrix-row f-matrix2 1)  (matrix-row f-matrix2 2)))
  (print (format nil "f-matrix = ~A" f-matrix))

  ;;momentベクトルをくっつける
  (setq m1 moment1)
  (setq m2 moment2)
  (setq m (float-vector (elt m1 0) (elt m1 1)  (elt m1 2) (elt m2 0) (elt m2 1)  (elt m2 2)))
  (print (format nil "moment  = ~A" m))

  (setq f-inverse-matrix (pseudo-inverse f-matrix))
  (print (format nil "f-inverse-matrix = ~A" f-inverse-matrix))

  ;; (setq p (scale -1 (simultaneous-equation f-matrix m)))
  ;; (print (format nil "p=~A with pseudo-inverse" p))

  (setq det (determinant f-matrix))
  (if (< det 0.5)
      (progn
        (print (format nil "inverse is not exist : determinant = ~A" det))
        nil)
    (progn
      (print (format nil "determinant = ~A" det))
      (setq p (scale -1 (transform f-inverse-matrix m)))
      p)
    )
  
  )



(defun rot-coords()
  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
  (setq pre-coords (send *robot* :rarm :end-coords :worldrot))
  (print (format nil "pre-coords = ~A" pre-coords))
  (send *robot* :rarm :end-coords :rot)

  (unix:sleep 1)
  (snap 0 90 0)
  (unix:sleep 1)  
  (setq post-coords (send *robot* :rarm :end-coords :worldrot))
  (print (format nil "post-coords = ~A" post-coords))
  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
  
  (send *robot* :rarm :end-coords :worldrot)

)
  ;; ;;何も持っていない状態でのforce,momentを記録
  ;; (prepare-plot :fname "pred_mass_JAXON_RED_2020_09-12_15-45_33")
  ;; (setq force-free (get :target "force" :period 2 :duration 1 ))
  ;; (setq moment-free (get :target "moment" :period 2 :duration 1 ))

  ;; (prepare-plot :create nil :fname "contact_70_JAXON_RED_2020_09-08_22-00_27")
  ;; (setq force-free (get :target "force" :period 2 :duration 1 ))
  ;; (setq moment-free (get :target "moment" :period 2 :duration 1 ))

    
