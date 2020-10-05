
;;predict.lのときは１姿勢を使って推定しようとしていたが、それでは歪対称行列の行列式が常に０であるため、求められないことがわかった
;; このコードでは複数姿勢の力モーメントを回転させて重ねて、拘束過多の疑似逆行列を用いて推定を行う
;;cascoords-~~から移植した

(load "./predict.l")
(load "../euslisp/reach.l")

;; (setq f1 (float-vector 0 0 -10))
;; (setq f2 (float-vector 0 0 -10))

;; (setq m1 (float-vector 0 10 0))
;; (setq m2 (float-vector 0 10 0))


;;0921 あってるかわからない、座標系とか　計算結果はそれっぽくなってはいるけど
;;comを推定
(defun pred-multi-com
    (force1 moment1 force2 moment2 pre-coords post-coords &key (g  9.8))

  ;;make f-matrix

  ;;pre-coods座標系で絶対座標系で表現されたforce１を表現し直す
  (format t "coord1     = ~A~%" pre-coords)
  (format t "f1 (world) = ~A~%" force1)
  (setq f  (send pre-coords :inverse-rotate-vector force1))
  (setq m1 (send pre-coords :inverse-rotate-vector moment1))
  (format t "f1(coord1) = ~A~%" f)
  ;; (print (format nil "f1 = ~A" f))
  (setq f-matrix1 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
  (format t "f-matrix1 = ~A~%~%" f-matrix1)

  ;; (setq f (transform change-rot force2))
  ;;小島さんより
  ;;post-coods座標系で,絶対座標系で表現されたforce2を表現し直す
  (format t "coord2     = ~A~%" post-coords)
  (format t "f2 (world) = ~A~%" force2)
  ;; (setq f2  (send post-coords :inverse-rotate-vector force2))
  ;; (setq mm2 (send post-coords :inverse-rotate-vector moment2))
  (setq f  (send post-coords :inverse-rotate-vector force2))
  (setq m2 (send post-coords :inverse-rotate-vector moment2))
  (format t "f2(coord2) = ~A~%" f)
  ;; (print (format nil "f2 = ~A" f2))
  ;;pre-coodsに座標系が重なるようにベクトルを回転させる
  ;; (setq f  (send (send post-coords :transformation pre-coords :world) :inverse-rotate-vector f2))
  ;; (setq f  (send (send post-coords :transformation pre-coords) :inverse-rotate-vector f2))
  ;; (setq m2 (send (send post-coords :transformation pre-coords :world) :inverse-rotate-vector mm2))
  (format t "f2(coord1) = ~A~%" f)
  (setq f-matrix2 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
  (format t "f-matrix2 = ~A~%~%" f-matrix2)

  (setq m (float-vector (elt m1 0) (elt m1 1)  (elt m1 2) (elt m2 0) (elt m2 1)  (elt m2 2)))
  (print (format nil "moment  = ~A" m))

  ;;f-matrixをくっつける
  (setq f-matrix (matrix (matrix-row f-matrix1 0) (matrix-row f-matrix1 1)  (matrix-row f-matrix1 2) (matrix-row f-matrix2 0) (matrix-row f-matrix2 1)  (matrix-row f-matrix2 2)))
  (print (format nil "f-matrix = ~A" f-matrix))

  ;;momentベクトルをくっつける
  ;;絶対座標系で表現されたmoment１をpre-coods座標系で表現し直す
  ;; (setq m1 (send pre-coords :inverse-rotate-vector moment1))
  ;;小島さんより
  ;; (setq m2 (transform change-rot moment2))
  ;;絶対座標系で表現されたmoment2をpost-coods座標系で表現し直す
  ;; (setq mm2 (send post-coords :inverse-rotate-vector moment2))
  ;;pre-coodsに座標系が重なるようにベクトルを回転させる  
  ;; (setq m2 (send (send post-coords :transformation pre-coords :world) :rotate-vector mm2))
  ;; (setq m (float-vector (elt m1 0) (elt m1 1)  (elt m1 2) (elt m2 0) (elt m2 1)  (elt m2 2)))
  ;; (print (format nil "moment  = ~A" m))

  (setq f-inverse-matrix (pseudo-inverse f-matrix))
  (format t "f-inverse-matrix = ~A~%" f-inverse-matrix)

  (setq com (scale -1 (transform f-inverse-matrix m)))
  (format t "com=~A with pseudo-inverse~%" com)
  (format t "com norm = ~A~%" (norm com))
  (setq com-world (send pre-coords :rotate-vector com))
  (format t "com-world =~A with pseudo-inverse~%" com-world)
  (format t "com-local norm = ~A~%" (norm com-world))
  com

)

(defun pred-triple-com
    (force1 moment1 force2 moment2 force3 moment3 pre-coords post-coords ppost-coords &key (g  9.8))

  ;; (setq change (get-transform-matrix pre-coords post-coords))
  ;; (setq change-rot (send change :worldrot))
  
  ;;make f-matrix

  ;;pre-coods座標系で絶対座標系で表現されたforce１を表現し直す
  (setq f (send pre-coords :inverse-rotate-vector force1))
  ;; (print (format nil "f1 = ~A" f))
  (setq f-matrix1 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
  (print (format nil "f-matrix1 = ~A" f-matrix1))

  ;; (setq f (transform change-rot force2))
  ;;小島さんより
  ;;post-coods座標系で,絶対座標系で表現されたforce2を表現し直す
  (setq f2 (send post-coords :inverse-rotate-vector force2))
  ;; (print (format nil "f2 = ~A" f2))
  ;;pre-coodsに座標系が重なるようにベクトルを回転させる
  (setq f (send (send post-coords :transformation pre-coords :world) :rotate-vector f2))
  (setq f-matrix2 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
  (print (format nil "f-matrix2 = ~A" f-matrix2))


  (setq f3 (send post-coords :inverse-rotate-vector force3))
  ;; (print (format nil "f3 = ~A" f3))
  ;;pre-coodsに座標系が重なるようにベクトルを回転させる
  (setq f (send (send ppost-coords :transformation pre-coords :world) :rotate-vector f3))
  (setq f-matrix3 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
  (print (format nil "f-matrix3 = ~A" f-matrix3))
  
  ;;f-matrixをくっつける
  (setq f-matrix (concatenate-matrix-column f-matrix1 f-matrix2 f-matrix3))
  
  (print (format nil "f-matrix = ~A" f-matrix))

  ;;momentベクトルをくっつける
  ;;絶対座標系で表現されたmoment１をpre-coods座標系で表現し直す
  (setq m1 (send pre-coords :inverse-rotate-vector moment1))
  ;; (setq m2 (transform change-rot moment2))
  ;;絶対座標系で表現されたmoment2をpost-coods座標系で表現し直す
  (setq mm2 (send post-coords :inverse-rotate-vector moment2))
  ;;pre-coodsに座標系が重なるようにベクトルを回転させる  
  (setq m2 (send (send post-coords :transformation pre-coords :world) :rotate-vector mm2))


  (setq mm3 (send ppost-coords :inverse-rotate-vector moment3))
  ;;pre-coodsに座標系が重なるようにベクトルを回転させる  
  (setq m3 (send (send ppost-coords :transformation pre-coords :world) :rotate-vector mm3))
  (setq m (concatenate float-vector m1 m2 m3))
  (print (format nil "moment  = ~A" m))

  (setq f-inverse-matrix (pseudo-inverse f-matrix))
  (print (format nil "f-inverse-matrix = ~A" f-inverse-matrix))

  (setq com (scale -1 (transform f-inverse-matrix m)))
  (print (format nil "com=~A with pseudo-inverse~%" com))
  (print (format nil "com norm = ~A" (norm com)))n
  ;; (setq com-local (send pre-coords :inverse-rotate-vector com))
  ;; (print (format nil "com-local =~A with pseudo-inverse" com-local))
  ;; (print (format nil "com-local norm = ~A" (norm com-local)))
  com

)

;; ;;小島さんのをそのままいれた感じ
;; (defun pred-multi-com
;;     (force1 moment1 force2 moment2 pre-coords post-coords &key (g  9.8) )

;;   ;; (setq change (get-transform-matrix pre-coords post-coords))
;;   ;; (setq change-rot (send change :worldrot))
  

;;   ;; (print (format nil "f1 = ~A" f))
;;   (setq f force1)
;;   (setq f-matrix1 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
;;   (print (format nil "f-matrix1 = ~A" f-matrix1))

;;   ;; (setq f (transform change-rot force2))
;;   ;;小島さんより
;;   ;;post-coods座標系で,絶対座標系で表現されたforce2を表現し直す
  
;;   ;; (setq f2 (send post-coords :inverse-rotate-vector force2))
;;   ;; (print (format nil "f2 = ~A" f2))
;;   ;;pre-coodsに座標系が重なるようにベクトルを回転させる
;;   (setq f (send (send post-coords :transformation pre-coords :world) :inverse-rotate-vector force2))
;;   (setq f-matrix2 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
;;   (print (format nil "f-matrix2 = ~A" f-matrix2))

;;   ;;f-matrixをくっつける
;;   (setq f-matrix (matrix (matrix-row f-matrix1 0) (matrix-row f-matrix1 1)  (matrix-row f-matrix1 2) (matrix-row f-matrix2 0) (matrix-row f-matrix2 1)  (matrix-row f-matrix2 2)))
;;   (print (format nil "f-matrix = ~A" f-matrix))

;;   ;;momentベクトルをくっつける
;;   ;;小島さんより
;;   ;; (setq m2 (transform change-rot moment2))
;;   (setq m1 moment2)
;;   (setq m2 (send (send post-coords :transformation pre-coords :world) :inverse-rotate-vector moment2))
;;   ;;pre-coodsに座標系が重なるようにベクトルを回転させる  
;;   ;; (setq m2 (send (send post-coords :transformation pre-coords :world) :rotate-vector mm2))
;;   (setq m (float-vector (elt m1 0) (elt m1 1)  (elt m1 2) (elt m2 0) (elt m2 1)  (elt m2 2)))
;;   (print (format nil "moment  = ~A" m))

;;   (setq f-inverse-matrix (pseudo-inverse f-matrix))
;;   (print (format nil "f-inverse-matrix = ~A" f-inverse-matrix))

;;   ;;comはlocal??
;;   (setq com (scale -1 (transform f-inverse-matrix m)))

;;   (setq com-local (send pre-coords :transform-vector com))
;;   (print (format nil "com=~A with pseudo-inverse" com))
;;   (print (format nil "com-local pre=~A with pseudo-inverse" com-local))

;;   (setq com-local (send post-coords :transform-vector com))
;;   (print (format nil "com-local post=~A with pseudo-inverse" com-local))
;;   (print (format nil "com norm = ~A" (norm com)))
;;   com

;; )


;;小島さんに教えてもらったメモたち0922
;; send (setq coords0 (make-coords :pos #f(100 0 0) :rpy (list (deg2rad 30) (deg2rad 30) 0))) :draw-on :flush t :size 100 :color #f(1 0 0)
;; send (setq coords1 (make-coords :pos #f(-100 0 0) :rpy (list (deg2rad 30) (deg2rad 60) 0))) :draw-on :flush t :size 100 :color #f(0 1 0)
;; (progn (send (send *irtviewer* :viewer) :draw-arrow #f(0 0 0) (send (send coords1 :transformation coords0 :world) :inverse-rotate-vector arrow)) (send *irtviewer* :viewer :flush))
;; (progn (send (send *irtviewer* :viewer) :draw-arrow #f(0 0 0) (send (send coords0 :transformation coords1 :world) :rotate-vector arrow)) (send *irtviewer* :viewer :flush))
;; (progn (send (send *irtviewer* :viewer) :draw-arrow #f(0 0 0) (send (send coords1 :transformation coords0 :world) :inverse-rotate-vector arrow) :color #f(0 0 1)) (send *irtviewer* :viewer :flush))


;;; pred-multi-comのテスト用、3バージョンの初期値に対して正しい値が出るかどうか
;;0922には全部正しく出た
(defun test-pred-multi-com
    (&key (d-init 30) (d 30))

  ;;setting ver1 ;;com=#f( 1 0 0)
  ;; (setq f1 (float-vector 0 0 -10))
  ;; (setq f2 (float-vector 0 0 -10))
  ;; (setq m1 (float-vector 0 10 0))
  ;; (setq m2 (float-vector (* -1 (* 10 (sin (deg2rad d)))) (* 10 (cos (deg2rad d))) 0))
  ;; (setq pre-coords (make-coords :pos #f(0  0 0) :rpy (list 0 0 0)))
  ;; (setq post-coords (make-coords :pos #f(100 0 0) :rpy (list (deg2rad d) 0 0)))

  ;; ver2 ;;com=#f( 1 0 0)
  ;; (setq f1 (float-vector 0 0 -10))
  ;; (setq f2 (float-vector 0 0 -10))
  ;; (setq m1 (float-vector 0 10 0))
  ;; (setq m2 (float-vector (* -1 (* 10 (sin (deg2rad d)))) (* 10 (cos (deg2rad d))) 0))
  ;; (setq pre-coords (make-coords :pos #f(100  0 0) :rpy (list 0 0 0)))
  ;; (setq post-coords (make-coords :pos #f(100 0 0) :rpy (list (deg2rad d) 0 0)))

  ;;ver3 ;;com=#f( 0 0 1) y軸方向に30度傾いた状態と更に30度y軸周りに回転させたとき
  (setq f1 (float-vector 0 0 -10))
  (setq f2 (float-vector 0 0 -10))
  (setq m1 (float-vector 0 (* 10 (sin (deg2rad d-init))) 0))
  (setq m2 (float-vector 0 (* 10 (sin (deg2rad (+ d-init d)))) 0))
  (setq pre-coords (make-coords :pos #f(100 0 0) :rpy (list 0 (deg2rad d-init) 0)))
  (setq post-coords (make-coords :pos #f(100 0 0) :rpy (list 0 (deg2rad (+ d-init d)) 0)))

  (setq com-local (pred-multi-com f1 m1 f2 m2 pre-coords post-coords))
  )

(defun test-pred-triple-com
    (&key (d-init 30) (d 30))

  ;;setting ver1 ;;com=#f( 1 0 0)
  ;; (setq f1 (float-vector 0 0 -10))
  ;; (setq f2 (float-vector 0 0 -10))
  ;; (setq m1 (float-vector 0 10 0))
  ;; (setq m2 (float-vector (* -1 (* 10 (sin (deg2rad d)))) (* 10 (cos (deg2rad d))) 0))
  ;; (setq pre-coords (make-coords :pos #f(0  0 0) :rpy (list 0 0 0)))
  ;; (setq post-coords (make-coords :pos #f(100 0 0) :rpy (list (deg2rad d) 0 0)))

  ;; ver2 ;;com=#f( 1 0 0)
  ;; (setq f1 (float-vector 0 0 -10))
  ;; (setq f2 (float-vector 0 0 -10))
  ;; (setq m1 (float-vector 0 10 0))
  ;; (setq m2 (float-vector (* -1 (* 10 (sin (deg2rad d)))) (* 10 (cos (deg2rad d))) 0))
  ;; (setq pre-coords (make-coords :pos #f(100  0 0) :rpy (list 0 0 0)))
  ;; (setq post-coords (make-coords :pos #f(100 0 0) :rpy (list (deg2rad d) 0 0)))

  ;;ver3 ;;com=#f( 0 0 1) y軸方向に30度傾いた状態と更に30度y軸周りに回転させたとき
  (setq f1 (float-vector 0 0 -10))
  (setq f2 (float-vector 0 0 -10))
  (setq f3 (float-vector 0 0 -10))
  (setq m1 (float-vector 0 (* 10 (sin (deg2rad d-init))) 0))
  (setq m2 (float-vector 0 (* 10 (sin (deg2rad (+ d-init d)))) 0))
  (setq m3 (float-vector 0 (* 10 (sin (deg2rad (+ d-init d 20)))) 0))
  (setq pre-coords (make-coords :pos #f(100 0 0) :rpy (list 0 (deg2rad d-init ) 0)))
  (setq post-coords (make-coords :pos #f(100 0 0) :rpy (list 0 (deg2rad (+ d-init d)) 0)))
  (setq ppost-coords (make-coords :pos #f(0 100 0) :rpy (list 0 (deg2rad (+ d-init d 20)) 0)))

  (setq com-local (pred-triple-com f1 m1 f2 m2 f3 m3 pre-coords post-coords ppost-coords))
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
;; #f(-0.285145 0.29928 0.p124284)

;;正しくは、
;;(send (car (send robot limb :force-sensors)) :rotate-vector (send self :off-moment-vector limb)) みたいにforce-sensors と:rotate-vectorを使わないと行けない（まだ書き直してない）
(defun test_on_simulator
    ()
  ;; (init)

  ;; (setq free-f-world (low-pass-online "force" "absolute"))

;;@red-multi-com実験のためコメントアウト
  (reach)

  (setq free-f-world (low-pass-online "force" "absolute"))
  (send *ri* :start-grasp)
  (init)
  (unix:sleep 1)
  
  (setq hold-f-world (low-pass-online "force" "absolute"))

  (init)
  (snap 0 10 0)

  ;; (setq hold-f-world (low-pass-online "force" "absolute"))
  
  (setq mass-vector-world (v- hold-f-world free-f-world))
  (print (format nil "mass-vector = ~A" mass-vector-world))
  (print (format nil "norm of mass-vector / 9.8 = ~A [kg]" (/ (norm mass-vector-world) 9.8)))
;;  ここまで

  ;;;
  (snap 0 -40 0)
  (unix:sleep 1)
  ;; (setq f1 (low-pass-online "force" "off"))
  ;; (setq m1 (low-pass-online "moment" "off"))
  (setq f1 (low-pass-online "force" "absolute"))
  (setq m1 (low-pass-online "moment" "absolute"))
  ;; (setq pre-coords (send *robot* :rarm-end-coords))
  (setq pre-coords (car (send *robot* :rarm :force-sensors)))
  
  (snap 0 20 0)
  (unix:sleep 1)
  ;; (setq f2 (low-pass-online "force" "off"))
  ;; (setq m2 (low-pass-online "moment" "off"))
  (setq f2 (low-pass-online "force" "absolute"))
  (setq m2 (low-pass-online "moment" "absolute"))
  ;; (setq post-coords (send *robot* :rarm-end-coords))
  (setq post-coords (car (send *robot* :rarm :force-sensors)))
  (setq com-local (pred-multi-com f1 m1 f2 m2 pre-coords post-coords)) ;;たぶんlocal

  ;; (snap 0 20 0)
  ;; (unix:sleep 1)
  ;; ;; (setq f2 (low-pass-online "force" "off"))
  ;; ;; (setq m2 (low-pass-online "moment" "off"))
  ;; (setq f3 (low-pass-online "force" "absolute"))
  ;; (setq m3 (low-pass-online "moment" "absolute"))
  ;; ;; (setq post-coords (send *robot* :rarm-end-coords))
  ;; (setq ppost-coords (car (send *robot* :rarm :force-sensors)))
  ;; (setq com-local (pred-triple-com f1 m1 f2 m2 f3 m3 pre-coords post-coords ppost-coords)) ;;たぶんlocal

;;pred-multi-comの実験のためコメントアウト  
  ;; (snap 0 66 0)
  ;; (unix:sleep 1)  
  
  ;; (setq contact-f-world (low-pass-online "force" "absolute"))
  ;; (setq contact-m-world (low-pass-online "moment" "absolute"))
  ;; (setq com-world (send (car (send *robot* :rarm :force-sensors)) :transform-vector com-local))
  ;; (setq right-side (v- contact-m-world (v* com-world mass-vector-world)))
  ;; (setq left-side-vector (v* com-world contact-f-world))
  ;; (setq tmp 0)
  ;; (dotimes (i 3)
  ;;   (setq tmp (+ tmp (/ (elt right-side i) (elt left-side-vector i))))
  ;;   )
  ;; (setq scalar (/ tmp 3.0))
  ;; (print (format nil "scalar = ~A" scalar))
  ;; (setq contact-point-world (scale scalar com-world))
  ;; (setq contact-point-local (send (car (send *robot* :rarm :force-sensors)) :inverse-transform-vector contact-point-world))
  ;; contact-point-local
;;ここまで
  )


;;plotをしてデータをみるべし
;; progn (setq x-list nil y-list nil z-list nil) (mapc #'(lambda (force) (push-back (elt force 0) x-list) (push-back (elt force 1) y-list) (push-back (elt force 2) z-list)) (setq force-list (mapcar #'(lambda (val) (unix:usleep 50000) (send *ri* :state :absolute-force-vector :rarm) ) (range 100)))) (average force-list)
;; (send *gp* :draw x-list y-list z-list)
;;呼ばれた瞬間から0.1秒間で5回分のセンサ値をlow-pass filterでならしてその平均を出力
(defun low-pass-online
    (target coords arm &key (times 40) (r 0.3) (sleep 10000) (flag nil))
  (let* (
         (y-data (list (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm)))
         (y-predict (list (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm)))
         )
    
    (dotimes (i times)
           (setq data (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm))
      (setq y (v+ (scale r data) (scale (- 1 r) (elt y-predict i))))
      (list-insert y (+ i 1) y-predict)
      (list-insert data (+ i 1) y-data)
      ;; (unix:usleep 1000)
      (unix:usleep sleep)      
      )
    (setq ret (cdr y-predict))
;;    (print (format nil "~A vector through filter = ~A" target ret))
    (setq ret-average (average ret))

    (if flag
        (list  (cdr y-data) ret)
      (progn
        (print (format nil "~A vector through filter and average = ~A" target ret-average))
        ret-average)
    )
    )
  )

;;filterをかけるとたしかに滑らかになった気がする
(defun compare-filter()
  
  (setq f1 (low-pass-online "force" "absolute" :rarm :flag t))

  (print (format nil "y-data= ~A" (elt f1 0)))

   (progn (setq x-list nil y-list nil z-list nil) (mapc #'(lambda (force) (push-back (elt force 0) x-list) (push-back (elt force 1) y-list) (push-back (elt force 2) z-list)) (elt f1 0)))
  (send *gp* :draw x-list y-list z-list)

  (unix:sleep 10)
  
  (progn (setq x-list-proc nil y-list-proc nil z-list-proc nil) (mapc #'(lambda (force) (push-back (elt force 0) x-list-proc) (push-back (elt force 1) y-list-proc) (push-back (elt force 2) z-list-proc)) (elt f1 1)))
  (send *gp* :draw x-list-proc y-list-proc z-list-proc)

  
  
)


;;座標系の変換関係でのシミュレーションでの実験
(defun rot-coords()
  (send (send *robot* :rarm :end-coords ) :draw-on :flush t :size 300)
  (setq pre-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (print (foxormat nil "pre-coords = ~A" pre-coords))
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

    
