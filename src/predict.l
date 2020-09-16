;;質量の推定
;;ここではグラフを見てfree の期間とholdの期間の開始時間を人が見て数字として与えている
;;各期間についてｚ方向の力を平滑化し、平均を取ってその差を取ることでz方向の力（期間のとり方によっては質量）を推定する
;;send (elt *rs-list* 2) :get (read-from-string (format nil ":rarm-force-vector" )


;;git checkout
;;データがある場所で
;;export ROBOT=JAXON_RED
;;roseus
;;load "~/research/rm_debris/predict.l"

(load "package://control_tools/log-tools.l")

(unless (and (boundp '*robot*) (boundp '*rs-list*)))

(unless (boundp '*rs-list*)  
  ;; (prepare-plot :fname "pred_mass_JAXON_RED_2020_09-12_15-45_33")
;;  (setq mass-vector (predict-mass))
  (prepare-plot :create t)
  (setq rs (car *rs-list*))
  )


;;plot用　0912時点ではplot関数がうまく行っていないから使ってない
(setq state-data-free nil)
(setq state-data-hold nil)
(setq state-time-data nil)

;;predict force 
;;input
;;target: 力がほしいなら "force" モーメントがほしいなら "moment"
;;free: ログのうち利用したい、何も持っていない区間の開始時間
;;hold; ログのうち利用したい、物を持っている区間の開始時間
;;output
;;force vector
(defun predict
    (&key (target "force") (free 2) (hold 10) (axis :x) (rs-list *rs-list*) (limbs (get-limbs-in-force-sensor-order *robot*)))
  (let* ((state-data
          (mapcar #'(lambda (l)
                       (mapcar #'(lambda (x) (send x :get (read-from-string (format nil "~A-off-~A-vector" l target)))) rs-list))
                  ;;ロボットモデルが古くて、座標系の変換が必要だったときの変換込みのもの
                  ;; (mapcar #'(lambda (rs)
                  ;;                 ;; todo move *robot*'s root coords
                  ;;                 (send *robot* :angle-vector (send rs :angle-vector))
                  ;;                 (send (car (send *robot* l :force-sensors)) :inverse-rotate-vector
                  ;;                       (send rs :get (read-from-string (format nil "~A-off-~A-vector" l target))))) rs-list))

                  (list :rarm)))
         ;;time-listn
         (state-time-data
          (mapcar #'(lambda (x) (send x :time )) rs-list)))

    ;;get index of free period and hold period
    (setq free-start-index (position-if #'(lambda (x) (and (< (- free 0.1) x) (< x (+ free 0.1) ))) state-time-data ))
    (setq free-end-index (position-if #'(lambda (x) (and (< (- (+ free 1) 0.1) x) (< x (+ (+ free 1) 0.1) ))) state-time-data ))
    
    (setq hold-start-index (position-if #'(lambda (x) (and (< (- hold 0.1) x) (< x (+ hold 0.1) ))) state-time-data ))
    (setq hold-end-index (position-if #'(lambda (x) (and (< (- (+ hold 1) 0.1) x) (< x (+ (+ hold 1) 0.1) ))) state-time-data ))

    ;;get list of each period
    (setq state-free (subseq (car state-data) free-start-index free-end-index))
    (setq state-hold (subseq (car state-data) hold-start-index hold-end-index))    

    ;;low-pass filter smoothing
    (setq state-free-smoothed (low-pass state-free))
    (setq state-hold-smoothed (low-pass state-hold))

    ;;without low-pass filter
    ;; (setq state-free-smoothed state-free)
    ;; (setq state-hold-smoothed state-hold)

    (setq pred (v- (average state-hold-smoothed) (average state-free-smoothed)))
    (print (format nil "~A = ~A"  target pred))
    ;; (print pred)
    pred
    )
  )


;;reach.lが最新
(defun display-arrow ()
  (do-until-key
   (unix:usleep 30)
   (send (send *irtviewer* :viewer) :draw-arrow (send *robot* :rarm :end-coords :worldpos) (scale 100 (send *ri* :off-force-vector :rarm)))
   )
  (send *robot* :rarm :end-coords :worldpos)
  )


;;low-pass filter
;;reffrence http://www.jsk.t.u-tokyo.ac.jp/~nozawa/share/doc/Control.pdf
(defun low-pass
    (state-data) ;; (&key r 0.3))
  (let* ((len (list-length state-data))
         (y-predict (list (elt state-data 0)))
         (r 0.3)
         )
    (dotimes (i len)
      (setq y (v+ (scale r (elt state-data i)) (scale (- 1 r) (elt y-predict i))))
      (list-insert y (+ i 1) y-predict)
      )
    (setq ret (cdr y-predict))
    (print (list-length ret))
    ret    
    )
  )

;;力の平均
;;average lis :axis 2とするとz軸方向の力の平均値が出る
;;average lis　だけだと力のベクトルがでる
(defun average
    (lis &key  (axis 10))
  (let* ((len (list-length lis))
         ;; (sum (list 0 0 0)))
         (sum_x 0)
         (sum_y 0)
         (sum_z 0))
         
    (dotimes (i len)
      (setq sum_x (+ sum_x (elt (elt lis i) 0)))
      (setq sum_y (+ sum_y (elt (elt lis i) 1)))
      (setq sum_z (+ sum_z (elt (elt lis i) 2)))      
      )
        
    (setq ave (float-vector (/ sum_x (float len)) (/ sum_y (float len)) (/ sum_z (float len))))

    ;; (print (format nil "ave = ~A" ave))
    ;; (print len)
    (cond
     ((equal axis 0)
      (float (elt ave 0)))
     ((equal axis 1)
      (elt ave 1))
     ((equal axis 2)
      (float (elt ave 2)))
     (t (progn (print "output vector")
               ave))
     )

    )
  )

;;plot したい
;;うまく行ってない
(defun plot-graph
    (&key data-list state-data-free) ;; time-list)
  (send *gp* :command "")
        (graph-view
         ;; (list ref-cmz-list act-cmz-list)
         (list data-list)
         state-time-data
         :keylist (list "original")
         :title "test"
         ;; :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
;         :dump-graph save
         :xlabel "Time[s]"
         :ylabel "CMz[mm]"
         :mode "lines"
         )

  )

;; (defun prepare
;;     ()
;;   (setq *log* (create-log-interface fname-with-path :robot robot))
;;predict mass return vector
(defun predict-mass
    (&key (g  9.8) (free 1) (hold 4.5))
  (setq mass-vector (predict :target "force" :free free :hold hold))
  (setq mass (* -1 (* (/ 1.0 g)  (elt mass-vector 2))))
  (print (format nil "mass = ~A " mass ))
  mass-vector
  )

;;棒なしから棒を掴んだときの差より
(defun predict-com
    (mass-vector &key (g  9.8) (free 1) (hold 4.5))
  ;massを求めているのと同じことしてる、、実はpredict-massでよい
  (setq f (predict :target "force" :free free :hold hold))
  (print (format nil "f = ~A" f))

  ;;モーメントのさを求める
  (setq m (predict :target "moment" :free free :hold hold))
  (print (format nil "m = ~A" m))

  (setq f-matrix (matrix (list 0 (- 0 (elt f 2)) (elt f 1)) (list (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (list (- 0 (elt f 1)) (elt f 0) 0)))
  (print (format nil "f-matrix = ~A" f-matrix))
  
  (setq p (scale -1 (transform (inverse-matrix f-matrix) m)))
  ;;  (norm p)
  p
  )


(defun predict-com-test
    (mass-vector f-row m &key (g  9.8) (free 1) (hold 4.5))
  (setq f (v- f-row mass-vector))
  (setq f-matrix (matrix (list 0 (- 0 (elt f 2)) (elt f 1)) (list (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (list (- 0 (elt f 1)) (elt f 0) 0)))
  (print (format nil "f-matrix = ~A" f-matrix))
  
  (setq p (scale -1 (transform (inverse-matrix f-matrix) m)))
  p
  )

;; 関数計算があっているか確認    あってそう？？？？
;; 34.irteusgl$ predict-cop-test a (float-vector 1 0 -1) (float-vector 0 10 0)
;; "f-matrix = #2f((0.0 1.0 0.0) (-1.0 0.0 -1.0) (0.0 1.0 0.0))"
;; #f(10.0 0.0 0.0)

(defun predict-contact-point
    ()
  ;;棒を掴んでないとき、掴んでいるときの差からmassと、comを求める
  (prepare-plot :fname "pred_mass_JAXON_RED_2020_09-12_15-45_33")
  (setq mass-vector (predict-mass :free 1 :hold 4.5) )
  (setq com (predict-com mass-vector :free 1 :hold 4.5))

  ;;ぶつけてみたときの接触点を求める
  ;; (prepare-plot :create nil :fname "contact_70_JAXON_RED_2020_09-08_22-00_27")
  ;; (setq com (predict-com mass-vector :free 2 :hold 10))
  (print (format nil "com = ~A" com))
  )

  
;; (setq mass-vector (predict-mass :free 2 :hold 10) )
;; (prepare-plot :create t)
;; (predict-cop mass-vector)
