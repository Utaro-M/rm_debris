;;質量の推定
;;ここではグラフを見てfree の期間とholdの期間の開始時間を人が見て数字として与えている
;;各期間についてｚ方向の力を平滑化し、平均を取ってその差を取ることでz方向の力（期間のとり方によっては質量）を推定する
;;git checkout 
;;export ROBOT=JAXON_RED
;;roseus

(load "package://control_tools/log-tools.l")

;; (unless (and (boundp '*robot*) (boundp '*rs-list*))
(unless (boundp '*rs-list*)  
  (prepare-plot)
  (setq rs (car *rs-list*))
  )

;;plot用　0912時点ではplot関数がうまく行っていないから使ってない
(setq state-data-free nil)
(setq state-data-hold nil)
(setq state-time-data nil)

;;predict force 
;; (defun predict-mass
(defun predict-force    
    (&key (free 2) (hold 10) (axis :x) (rs-list *rs-list*) (limbs (get-limbs-in-force-sensor-order *robot*)))
  (let* ((state-data
          (mapcar #'(lambda (l)
                      (mapcar #'(lambda (x) (send x :get (read-from-string (format nil "~A-off-force-vector" l)))) rs-list))
                  (list :rarm)))
         ;;time-list
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
    
    ;;diff 
    (setq mass (- (ave state-hold-smoothed) (ave state-free-smoothed)))
    ;;print to check
    (print (format nil "free-start-index = ~A" free-start-index))
    (print (format nil "free-end-index = ~A" free-end-index))
    ;; (print (list-length (car state-data)))
    ;; (print (list-length state-free))
    ;; (print (list-length state-hold-smoothed))
    (print (format nil "mass = ~A" mass))
;;    state-free
    )
  )

;;(predict-mass)


;;low-pass filter  
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

;;z方向の平均
(defun ave (lis)
  (let* ((len (list-length lis))
             (sum 0))
    (dotimes (i len)
      (setq sum (+ sum (elt (elt lis i) 2)))
      )
    (setq ave (/ sum (float len)))
    (print (format nil "ave = ~A" ave))
    (/ ave 9.8)
    )
  )
;;plot したい
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
