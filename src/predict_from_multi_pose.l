
;;predict.lのときは１姿勢を使って推定しようとしていたが、それでは歪対称行列の行列式が常に０であるため、求められないことがわかった
;; このコードでは複数姿勢の力モーメントを回転させて重ねて、拘束過多の疑似逆行列を用いて推定を行う
;;cascoords-~~から移植した

(load "./predict.l")
(load "../euslisp/reach.l")

(setq f1 (float-vector 0 0 0))
(setq f2 (float-vector 0 0 -1))

(setq m1 (float-vector 0 0 0))
(setq m2 (float-vector 0 10 0))

;; (setq pre-coords  #<coordinates #X557a04797290  209.447 -418.273 1127.391 / -0.157 -0.093 -0.086>)
;; (setq post-coords  #<coordinates #X557a1f5c1dd0  209.465 -418.263 1127.399 / 1.102 1.275 1.245>)


(defun pred-multi-test
    (force1 moment1 force2 moment2 pre-coords post-coords &key (g  9.8))

  (setq change (get-transform-matrix pre-coords post-coords))
  (setq change-rot (send change :worldrot))
  
  ;;make f-matrix
  (setq f force1)
        
  (setq f-matrix1 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))

  (print (format nil "f-matrix1 = ~A" f-matrix1))
  (setq f (transform change-rot force2))
  (setq f-matrix2 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
  (print (format nil "f-matrix2 = ~A" f-matrix2))

  ;;f-matrixをくっつける
  (setq f-matrix (matrix (matrix-row f-matrix1 0) (matrix-row f-matrix1 1)  (matrix-row f-matrix1 2) (matrix-row f-matrix2 0) (matrix-row f-matrix2 1)  (matrix-row f-matrix2 2)))
  (print (format nil "f-matrix = ~A" f-matrix))

  ;;momentベクトルをくっつける
  (setq m1 moment1)
  (setq m2 (transform change-rot moment2))
  (setq m (float-vector (elt m1 0) (elt m1 1)  (elt m1 2) (elt m2 0) (elt m2 1)  (elt m2 2)))
  (print (format nil "moment  = ~A" m))

  (setq f-inverse-matrix (pseudo-inverse f-matrix))
  (print (format nil "f-inverse-matrix = ~A" f-inverse-matrix))

  ;;  (setq p (scale -1 (simultaneous-equation f-matrix m)))
  (setq p (scale -1 (transform f-inverse-matrix m)))
  (print (format nil "p=~A with pseudo-inverse" p))

  ;; (setq det (determinant f-matrix))
  ;; (if (< det 0.5)
  ;;     (progn
  ;;       (print (format nil "inverse is not exist : determinant = ~A" det))
  ;;       nil)
  ;;   (progn
  ;;     (print (format nil "determinant = ~A" det))
  ;;     (setq p (scale -1 (transform f-inverse-matrix m)))
  ;;     p)
  ;;   )
  
 ;; )
)


(defun rot-coords()
  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
  (setq pre-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (print (format nil "pre-coords = ~A" pre-coords))
  (send *robot* :rarm :end-coords :rot)

  (unix:sleep 1)
  (snap 0 90 0)
  (unix:sleep 1)  
  (setq post-coords (send *robot* :rarm :end-coords :copy-worldcoords))  
  (print (format nil "post-coords = ~A" post-coords))
;;  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)

  (send pre-coords :draw-on :flush nil :size 300)
  (send post-coords :draw-on :flush nil :size 300 :color #f(1 0 0))

  ;;postからpreへの変換行列を作る :worldで
  (setq change (send post-coords :transformation pre-coords :world))
  (send change :draw-on :flush nil :size 300 :color #f(0 0 1))
  (print (format nil "change = ~A" change))
  (print (format nil "angle = ~A" (send change :euler-angle)))

  (setq post (float-vector 1 1 1))
  (setq p (send post-coords :inverse-transform-vector post))
  (setq ret-vec (send (send *irtviewer* :viewer) :draw-arrow (send post-coords :worldpos) (send (send post-coords :translate (scale 100 p) ) :worldpos)))
  ;; (setq ret-vec (transform  (send change :worldrot) post))
  
  (setq ret (transform-coords change post-coords))
  (send ret :draw-on :flush t :size 300 :color #f(0 1 0))
  (print (format nil "ret = ~A" ret))
)



(defun get-transform-matrix
    (pre-coords post-coords)
  ;;postからpreへの変換行列を作る :worldで
  (setq change (send post-coords :transformation pre-coords :world))
  (send change :draw-on :flush nil :size 300 :color #f(0 0 1))
  (print (format nil "change = ~A" change))
  change
  )
  


;; (defun rot-coords()
;;   (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
;;   (setq pre-coords (send *robot* :rarm :end-coords :worldrot))
;;   (print (format nil "pre-coords = ~A" pre-coords))
;;   (send *robot* :rarm :end-coords :rot)

;;   (unix:sleep 1)
;;   (snap 0 90 0)
;;   (unix:sleep 1)  
;;   (setq post-coords (send *robot* :rarm :end-coords :worldrot))
;;   (print (format nil "post-coords = ~A" post-coords))
;;   (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
  
;;   (send *robot* :rarm :end-coords :worldrot)

;; )
  ;; ;;何も持っていない状態でのforce,momentを記録
  ;; (prepare-plot :fname "pred_mass_JAXON_RED_2020_09-12_15-45_33")
  ;; (setq force-free (get :target "force" :period 2 :duration 1 ))
  ;; (setq moment-free (get :target "moment" :period 2 :duration 1 ))

  ;; (prepare-plot :create nil :fname "contact_70_JAXON_RED_2020_09-08_22-00_27")
  ;; (setq force-free (get :target "force" :period 2 :duration 1 ))
  ;; (setq moment-free (get :target "moment" :period 2 :duration 1 ))

    
