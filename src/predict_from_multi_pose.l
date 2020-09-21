
;;predict.lのときは１姿勢を使って推定しようとしていたが、それでは歪対称行列の行列式が常に０であるため、求められないことがわかった
;; このコードでは複数姿勢の力モーメントを回転させて重ねて、拘束過多の疑似逆行列を用いて推定を行う
;;cascoords-~~から移植した

;;(load "./predict.l")
(load "../euslisp/reach.l")

;; (setq f1 (float-vector 0 0 -10))
;; (setq f2 (float-vector 0 0 -10))

;; (setq m1 (float-vector 0 10 0))
;; (setq m2 (float-vector 0 10 0))

;; (setq pre-coords  #<coordinates #X557a04797290  209.447 -418.273 1127.391 / -0.157 -0.093 -0.086>)
;; (setq post-coords  #<coordinates #X557a1f5c1dd0  209.465 -418.263 1127.399 / 1.102 1.275 1.245>)

;;comを推定
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
  (setq com (scale -1 (transform f-inverse-matrix m)))
  (print (format nil "com=~A with pseudo-inverse" com))
  (print (format nil "com norm = ~A" (norm com)))
  com

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



;;simulator で重心を探してみた
;;0921 log

;; test_on_simulator
;; Do you move real machine?(Y or N): y
;; ;; #<rotational-joint #X55895c89dfe0 CHEST_JOINT1> :joint-angle(-0.053159) violate min-angle(0.0)
;; "change = #<coordinates #X55896bfef2d8  2.842e-14 -5.684e-14 0.0 / 0.0 -1.691e-17 0.0>"
;; "f-matrix1 = #2f((0.0 1.35645 -18.9717) (-1.35645 0.0 -12.8462) (18.9717 12.8462 0.0))"
;; "f-matrix2 = #2f((0.0 -10.5833 -11.2803) (10.5833 0.0 -22.9626) (11.2803 22.9626 0.0))"
;; "f-matrix = #2f((0.0 1.35645 -18.9717) (-1.35645 0.0 -12.8462) (18.9717 12.8462 0.0) (0.0 -10.5833 -11.2803) (10.5833 0.0 -22.9626) (11.2803 22.9626 0.0))"
;; "moment  = #f(1.56924 4.48874 3.16529 7.62513 2.85227 -3.11992)"
;; "f-inverse-matrix = #2f((-0.025151 -0.020498 0.048508 0.019029 0.022899 -0.016881) (0.019979 0.014639 -0.015537 -0.024534 -0.012644 0.039754) (-0.022482 -0.015974 0.010511 -0.003976 -0.014085 -0.006385))"
;; "com=#f(-0.285145 0.29928 0.124284) with pseudo-inverse"
;; "com norm = 0.431651"
;; #f(-0.285145 0.29928 0.124284)
(defun test_on_simulator
    ()
  ;; (init)
  ;; (grab_on_table)
  ;; (init)
  (setq f1 (send *ri* :off-force-vector :rarm))
  (setq m1 (send *ri* :off-moment-vector :rarm))
  (setq pre-coords (send *robot* :rarm-end-coords))
  
  (snap 0 30 0)
  
  (setq f2 (send *ri* :off-force-vector :rarm))
  (setq m2 (send *ri* :off-moment-vector :rarm))
  (setq post-coords (send *robot* :rarm-end-coords))
  (pred-multi-test f1 m1 f2 m2 pre-coords post-coords)
  )

;; ;;
;; (setq pos_rarm (scale 0.001 (send *robot* :rarm :end-coords :worldpos)))
;; (setq pos_stick #f(0.259 -0.425 1.174))
;; (setq com_simulator (v- pos_stick pos_rarm))
;; (norm com_simulator)
      
      

;;座標系の変換関係でのシミュレーションでの実験
(defun rot-coords()
  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
  (setq pre-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (print (format nil "pre-coords = ~A" pre-coords))
  (send *robot* :rarm :end-coords :rot)

  (unix:sleep 1)
  (snap 0 60 0)
  (unix:sleep 1)  
  (setq post-coords (send *robot* :rarm :end-coords :copy-worldcoords))  
  (print (format nil "post-coords = ~A" post-coords))
;;  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)

  (send pre-coords :draw-on :flush nil :size 300)
  (send post-coords :draw-on :flush nil :size 300 :color #f(1 0 0))

  ;;postからpreへの変換行列を作る :worldで

  ;;関数を作った
  (setq change get-transform-matrix pre-coords post-coords)
  ;;not 関数
  ;; (setq change (send post-coords :transformation pre-coords :world))
  ;; (send change :draw-on :flush nil :size 300 :color #f(0 0 1))
  ;; (print (format nil "change = ~A" change))

  ;; (setq post (float-vector 1 1 1))
  ;; (setq p (send post-coords :inverse-transform-vector post))
  ;; (setq ret-vec (send (send *irtviewer* :viewer) :draw-arrow (send post-coords :worldpos) (send (send post-coords :translate (scale 100 p) ) :worldpos)))
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

    
