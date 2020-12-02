;;右手を下に
;;(progn (my-init-pose :move-foot? nil) (start-coords))
;;(setq rarm-coords (make-coords :pos (float-vector 500 -400 600) :rpy (float-vector (deg2rad 90) (deg2rad  -30) (deg2rad 0))))
;;(setq larm-coords (make-coords :pos (float-vector 500 400 900) :rpy (float-vector 0 (deg2rad  50) (deg2rad 90))))
;;(reach :rarm-coords rarm-coords :larm-coords larm-coords :send? nil)


;;(progn (my-init-pose :move-foot? nil) (start-coords)
;; (setq rarm-coords (make-coords :pos (float-vector 540 -500 800) :rpy (float-vector (deg2rad 100) (deg2rad  20) (deg2rad 30))))
;; (setq larm-coords (make-coords :pos (float-vector 500 400 900) :rpy (float-vector 0 (deg2rad  60) (deg2rad 90))))
;; (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t)
;; (heave #f(100 0 0) #f(0 0 0) :reset? t :fix-contact? 2 :send? nil))



;;sort
(defun sort-up(lis)
  (dotimes (i (length lis))
    (dotimes j
(setq idx 0) (setq val 0) (setq item (elt (elt lis i) 0)) (if (> val item ) (progn (setq tmp item) (list-insert (elt lis i) idx lis)(setq item val) (setq val tmp)))

)))
(defun quick-sort(seq)
    (if (< (length seq) 1)
      (return-from quick-sort seq))
  (let ((pivot (elt seq 0))
        ( left nil)
        ( right nil))

  (dotimes (i (- (length seq) 1))
    ;; (print i)
    (if (< (elt seq (+ i 1)) pivot)
        (setq left (list-insert (elt seq (+ i 1)) (length seq) left))
      (setq right (list-insert (elt seq (+ i 1)) (length seq) right))
      ))
  ;; (format t "left = ~A~%" left)
  ;; (format t "right = ~A~%" right)
  (setq left (quick-sort left))
  (setq right (quick-sort right))
  (setq fwd (append left (list pivot) ))
  (setq ret (append fwd right ))
  ret
  ))

(defun quick-sort-vector-list(seq)
  (if (< (length seq) 2)
      (return-from quick-sort-vector-list seq))
  (let ((pivot (elt (elt seq 0) 0))
        (pivot-vec (elt seq 0) )
        (left nil)
        (right nil))

  (dotimes (i (- (length seq) 1))
    (if (< (elt (elt seq (+ i 1)) 0) pivot)
        (setq left (list-insert (elt seq (+ i 1)) (length seq) left))
      (setq right (list-insert (elt seq (+ i 1)) (length seq) right))
      ))
  ;; (format t "left = ~A~%" left)
  ;; (format t "right = ~A~%" right)
  (setq left (quick-sort-vector-list left))
  (setq right (quick-sort-vector-list right))
  (setq fwd (append left (list pivot-vec)))
  (setq ret (append fwd right ))
  ret
  ))

;;listのうちx要素が小さいものをnumこ取り出す
(defun get-min-val(lis num)
  (setq ret-lis (quick-sort-vector-list lis))
  (subseq ret-lis 0 num))

;;plane　オブジェクト(prismなど）をコピーして渡す
(defun get-pose(contact-wall arm)
  (let* ((idx-x 0)
        (idx-y 0)
        (longest-len 0)
        (vertices-list (get-min-val (send contact-wall :vertices) 4))
        (vertices-list-reverse (reverse vertices-list))
        (plane-normal (send (elt (send contact-wall :faces) 0) :normal)))
  (dotimes (i (length vertices-list))
    (dotimes (j (- (length vertices-list) i))
      (setq len-tmp (norm (v- (elt vertices-list i) (elt vertices-list-reverse j))))
      (if (< longest-len len-tmp)
          (progn
            (setq idx-x i)
            (setq idx-y j) ;;reverseされているがそのままいく
            (setq longest-len len-tmp)))))
  (setq longest-edge (make-line (elt vertices-list idx-x) (elt vertices-list-reverse idx-y)))

  (setq normal-line (make-line #f(0 0 0) plane-normal))
  ;;手先の目標
  (setq x-vec (car (send longest-edge :common-perpendicular normal-line))) ;;x-vec
  (setq y-vec  plane-normal)
  (setq z-vec (v- (elt vertices-list idx-y) (elt vertices-list-reverse idx-x)))
  ;;x-vecは正面方向
  (if (< (elt x-vec 0) 0)
      (setq x-vec (scale -1 x-vec)))

  (if (equal arm "larm")
      (progn
        (if (< (elt y-vec 2) 0)
          (setq y-vec (scale -1 y-vec)))
        (if (> (elt z-vec 1) 0)
            (setq z-vec (scale -1 z-vec))))
    (progn
      (if (> (elt y-vec 2) 0)
        (setq y-vec (scale -1 y-vec)))
      (if (< (elt z-vec 1) 0)
          (setq z-vec (scale -1 z-vec)))))

  (setq x-vec (normalize-vector x-vec))
  (setq y-vec (normalize-vector y-vec))
  (setq z-vec (normalize-vector z-vec))
  (format t "plane-normal = ~A~%" plane-normal)
  (format t "x-vec = ~A~%" x-vec)
  (format t "y-vec = ~A~%" y-vec)
  (format t "z-vec = ~A~%" z-vec)
  (setq rot (transpose (matrix x-vec y-vec z-vec)))
  rot
  ))
    
      
  ;; (if (< (elt plane-normal 2) 0)
  ;;     (setq plane-normal (scale -1 plane-normal))) ;;normalはz正方向に揃える
    
;;output flag   0 ::both arms holding , 1 ::rarm drop , 2 ::larm drop , 3 ::both arms drop, 10 ::finish smoothly
(defun check-force
    (&key (checking-time 6) (sleep-time 20000) (working? t) (thre-list (list 22 22)) (fin 3))
  (setq initial (send *robot* :angle-vector)
        rarm-f-pre  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil)
        larm-f-pre  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil)
        angle-pre (send *ri* :state :angle-vector)
        flag 0)

  (if working? 
      (format t "~%~%check while working~%")
    (format t "~%~%check while still~%"))
  
  (dotimes (i (/ (* checking-time 1000000) sleep-time))
    (setq rarm-f  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil))
    (setq larm-f  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil))
    ;;絶対値バージョン
    ;; (setq rarm-f-dif (abs (elt (v- rarm-f rarm-f-pre) 2)))
    ;; (setq larm-f-dif (abs (elt (v- larm-f larm-f-pre) 2)))
    (setq rarm-f-dif (elt (v- rarm-f rarm-f-pre) 2))
    (setq larm-f-dif (elt (v- larm-f larm-f-pre) 2))
    (setq drop (list nil nil))
    (format t "rarm-f-dif       = ~A~%" rarm-f-dif)
    (format t "larm-f-dif       = ~A~%" larm-f-dif)

    ;;drop判定
    (if (>  larm-f-dif (* +1 (elt thre-list 1)))
        (setq flag (+ flag 1)))
    (if (>  rarm-f-dif (* +1 (elt thre-list 0)))
        (setq flag (+ flag 2)))

    (cond flag
          (0
           (progn
             (print "holding")
             (return 0))           
           )
          (1
           (progn
             (print "drop larm")
             (return 1)))
          (2
           (progn
             (print "drop rarm")
             (return 2)))
          (3
           (progn
                (print "drop both arm")
                (setq flag 0)
                (return 3)))
          )
          
    
    ;; (if (>  rarm-f-dif (* +1 (elt thre-list 0)))
    ;;     (setq drop (list-insert t 0 drop)))

    ;; (if (>  larm-f-dif (* +1 (elt thre-list 1)))
    ;;     (setq drop (list-insert t 1 drop)))

    ;; (if (or (elt drop 0) (elt drop 1))
    ;;     (progn
    ;;       (print "drop")
    ;;       (send *ri* :stop-motion)
    ;;       (print "stop motion")
    ;;       (if (and (elt drop 0) (elt drop 1))
    ;;           (progn
    ;;             (print "drop both arm")
    ;;             (setq flag 0)
    ;;             (return 0))
    ;;         (progn
    ;;           (cond ((elt drop 0)
    ;;                  (progn
    ;;                    (print "drop rarm")
    ;;                    (setq flag 1)
    ;;                    (return 1)))
    ;;                 ((elt drop 1)
    ;;                  (progn
    ;;                    (print "drop larm")
    ;;                    (setq flag 2)
    ;;                    (return 2))))))))

    (if working?
        (progn
          ;;動作終了判定
          (setq tmp (send *ri* :reference-vector))
          (setq angle-vector-dif (abs (norm (v- initial tmp))))
          (format t "angle-vector-dif = ~A~%" angle-vector-dif)
          (if (< angle-vector-dif 0.5)
              (progn
                (print "finish")
                (setq flag 10)
                (return angle-vector-dif))
            )
          (setq rarm-f-pre rarm-f)
          (setq larm-f-pre larm-f)))
    
    (setq angle-pre (send *ri* :state :angle-vector))
    (unix:usleep sleep-time)
    )
  (format t "flag = ~A~%" flag)
  flag
  )

;;output flag   0 ::both arms holding , 1 ::rarm drop , 2 ::larm drop , 3 ::both arms drop, 10 ::finish smoothly
(defun check-pcl()
  (setq drop_flag_larm (one-shot-subscribe "/drop_flag_larm" std_msgs::Float64 :timeout 10000))
  (setq drop_flag_rarm (one-shot-subscribe "/drop_flag_rarm" std_msgs::Float64 :timeout 10000))
  (if (not drop_flag_larm)
      (setq flag 2))
  (if (not drop_flag_rarm)
      (setq flag 1))
  (if (and (not drop_flag_larm) (not drop_flag_rarm))
      (setq flag 3))
  )

;; (defun get-half-plane(plane &key (offset 500))
;;   (setq origin-face plane)
;;   (setq vertices-list (send origin-face :vertices))
;;   (setq com (send origin-face :centroid))
;;   (setq len-lis (length vertices-list))
;;   (setq new-vertices-list (mapcar #'(lambda (l) (progn (if (< (elt l 0) (+ (elt com 0) offset)) l))) vertices-list))
;;   (setq new-vertices (intersection new-vertices-list vertices-list :test #'equal))
;;   ;; (setq latest-vertices (list-insert com len-lis new-vertices))

;;   ;; (setq norm-list (mapcar #'(lambda (l) (progn (norm (v- l com)))) new-vertices))
;;   ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (car (quick-sort norm-list)) new-vertices))) 1) new-vertices))
;;   ;; (setq latest-vertices  new-vertices)
;;   (setq i 0)
;;   (setq latest-vertices (flatten (mapcar #'(lambda (l) (progn (setq tmp (list (scale 0.5 (v+ (elt new-vertices i) l)) l)) (setq i (+ i 1)) (return tmp))) (cdr new-vertices))))
;;   ;; (setq norm-list (mapcar #'(lambda (l) (progn (norm (v- l com)))) new-vertices))

;;   ;; (setq i 0)
;;   ;; (setq norm-list (mapcar #'(lambda (l) (progn (setq tmp (abs (norm (v- (elt latest-vertices i) l)))) (setq i (+ i 1)) (return tmp))) ;; (cdr latest-vertices)
;;   ;;                         (list-insert (car latest-vertices) 300 (cdr latest-vertices))))

  
;;   ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (last (quick-sort norm-list)) latest-vertices))) 0) latest-vertices))
;;   (setq max-val (last (quick-sort-vector-list latest-vertices)))
;;   (format t "max-val =~A~%" max-val)
;;   ;; (format t "latest-vertices = ~A~%" latest-vertices)
;;   (setq latest-vertices (list-insert com (- (- (length latest-vertices) (length (member max-val latest-vertices))) -1) latest-vertices))
;;   ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (car (quick-sort norm-list)) new-vertices))) 1) new-vertices))
;;   ;; (setq latest-vertices (list-insert (car latest-vertices) 300 latest-vertices))
;;   (setq face-arm (make-prism latest-vertices 10))
;;   (send face-arm :set-color (float-vector 1 0 0))
;;   (objects (list face-arm origin-face))
;;   (format t "com = ~A~%" com)
;;   ;; (format t "origin vertices = ~A~%" vertices-list)
;;   (format t "latest vertices = ~A~%" latest-vertices)
;;   )
;;plane = object
;;bimyounatokimoaru
(defun get-half-plane(obj &key (offset 500))
  (setq origin-obj (copy-object obj))
  (setq origin-vertices (send (car (send origin-obj :faces)) :vertices))
  (setq com (send origin-obj :centroid))
  (setq len-lis (length origin-vertices))
  (setq back (mapcar #'(lambda (l) (progn (if (< (elt l 0) (+ (elt com 0) offset)) l))) origin-vertices))
  (setq back-vertices (intersection back origin-vertices :test #'equal))
  ;; (setq back-obj (make-prism (mapcar #'(lambda (l) (scale 1.1 l)) back-vertices) 10))
  (format t "back vertices = ~A~%" back-vertices)
  (setq back-obj (make-prism (cdr back-vertices) 20))
  (send back-obj :set-color (float-vector 0 1 0))
  ;; (send origin-obj :set-color (float-vector 0 0 1))  
  (format t "origin-obj = ~A~%" origin-obj)
  (format t "back-obj = ~A~%" back-obj)
  (objects (list back-obj origin-obj))
  ;; (setq front-obj (body- origin-obj back-obj))
  ;; (send front-obj :set-color (float-vector 1 0 0))
  ;; (objects (list front-obj origin-obj))
  )
(defun tmp (obj &key (offset 500))
  ;; (setq latest-vertices (list-insert com len-lis new-vertices))

  ;; (setq norm-list (mapcar #'(lambda (l) (progn (norm (v- l com)))) new-vertices))
  ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (car (quick-sort norm-list)) new-vertices))) 1) new-vertices))
  ;; (setq latest-vertices  new-vertices)
  (setq origin-obj (copy-object obj))
  (setq origin-vertices (send (car (send origin-obj :faces)) :vertices))
  (setq i 0)
  (setq latest-vertices (flatten (mapcar #'(lambda (l) (progn (setq tmp (list (scale 0.5 (v+ (elt origin-vertices i) l)) l)) (setq i (+ i 1)) (return tmp))) (cdr origin-vertices))))
  ;; (setq norm-list (mapcar #'(lambda (l) (progn (norm (v- l com)))) new-vertices))

  ;; (setq i 0)
  ;; (setq norm-list (mapcar #'(lambda (l) (progn (setq tmp (abs (norm (v- (elt latest-vertices i) l)))) (setq i (+ i 1)) (return tmp))) ;; (cdr latest-vertices)
  ;;                         (list-insert (car latest-vertices) 300 (cdr latest-vertices))))

  
  ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (last (quick-sort norm-list)) latest-vertices))) 0) latest-vertices))
  (setq max-val (last (quick-sort-vector-list latest-vertices)))
  (format t "max-val =~A~%" max-val)
  ;; (format t "latest-vertices = ~A~%" latest-vertices)
  (setq latest-vertices (list-insert com (- (- (length latest-vertices) (length (member max-val latest-vertices))) -1) latest-vertices))
  ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (car (quick-sort norm-list)) new-vertices))) 1) new-vertices))
  ;; (setq latest-vertices (list-insert (car latest-vertices) 300 latest-vertices))
  (setq face-arm (make-prism latest-vertices 10))
  (send face-arm :set-color (float-vector 1 0 0))
  (objects (list face-arm origin-obj))
  (format t "com = ~A~%" com)
  ;; (format t "origin vertices = ~A~%" vertices-list)
  (format t "latest vertices = ~A~%" latest-vertices)
  )

    ;; (if (>  rarm-f-dif (* +1 (elt thre-list 0)))
    ;;     (setq drop (list-insert t 0 drop)))

    ;; (if (>  larm-f-dif (* +1 (elt thre-list 1)))
    ;;     (setq drop (list-insert t 1 drop)))

    ;; (if (or (elt drop 0) (elt drop 1))
    ;;     (progn
    ;;       (print "drop")
    ;;       (send *ri* :stop-motion)
    ;;       (print "stop motion")
    ;;       (if (and (elt drop 0) (elt drop 1))
    ;;           (progn
    ;;             (print "drop both arm")
    ;;             (setq flag 0)
    ;;             (return 0))
    ;;         (progn
    ;;           (cond ((elt drop 0)
    ;;                  (progn
    ;;                    (print "drop rarm")
    ;;                    (setq flag 1)
    ;;                    (return 1)))
    ;;                 ((elt drop 1)
    ;;                  (progn
    ;;                    (print "drop larm")
    ;;                    (setq flag 2)
    ;;                    (return 2))))))))

;; (defun check-while-working
;;     (&key (checking-time 6) (sleep-time 20000) (thre-list (list 22 22)) (fin 3))
;;   (format t "~%~%check while working~%")
;;   (setq initial (send *robot* :angle-vector)
;;         rarm-f-pre  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil)
;;         larm-f-pre  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil)
;;         angle-pre (send *ri* :state :angle-vector))
  
;;   (dotimes (i (/ (* checking-time 1000000) sleep-time))
;;     (setq rarm-f  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil))
;;     (setq larm-f  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil))
;;     ;;絶対値バージョン
;;     ;; (setq rarm-f-dif (abs (elt (v- rarm-f rarm-f-pre) 2)))
;;     ;; (setq larm-f-dif (abs (elt (v- larm-f larm-f-pre) 2)))
;;     (setq rarm-f-dif (elt (v- rarm-f rarm-f-pre) 2))
;;     (setq larm-f-dif (elt (v- larm-f larm-f-pre) 2))
;;     (setq drop (list nil nil))
;;     (format t "rarm-f-dif       = ~A~%" rarm-f-dif)
;;     (format t "larm-f-dif       = ~A~%" larm-f-dif)

;;     ;;drop判定
;;     (if (>  rarm-f-dif (* +1 (elt thre-list 0)))
;;         (setq drop (list-insert t 0 drop)))

;;     (if (>  larm-f-dif (* +1 (elt thre-list 1)))
;;         (setq drop (list-insert t 1 drop)))

;;     (if (or (elt drop 0) (elt drop 1))
;;         (progn
;;           (print "drop")
;;           (send *ri* :stop-motion)
;;           (print "stop motion")
;;           (if (and (elt drop 0) (elt drop 1))
;;               (progn
;;                 (print "drop both arm")
;;                 (setq flag 0)
;;                 (return 0))
;;             (progn
;;               (cond ((elt drop 0)
;;                      (progn
;;                        (print "drop rarm")
;;                        (setq flag 1)
;;                        (return 1)))
;;                     ((elt drop 1)
;;                      (progn
;;                        (print "drop larm")
;;                        (setq flag 2)
;;                        (return 2))))))))

;;     ;;動作終了判定
;;     (setq tmp (send *ri* :reference-vector))
;;     (setq angle-vector-dif (abs (norm (v- initial tmp))))
;;     (format t "angle-vector-dif = ~A~%" angle-vector-dif)
;;     (if (< angle-vector-dif 0.5)
;;         (progn
;;           (print "finish")
;;           (setq flag 10)
;;           (return angle-vector-dif))
;;       )
    
;;     (setq angle-pre (send *ri* :state :angle-vector))
;;     (setq rarm-f-pre rarm-f)
;;     (setq larm-f-pre larm-f)
;;     (unix:usleep sleep-time)
;;     )
;;   (format t "flag = ~A~%" flag)
;;   flag
;;   )


  
;; ;;静止時の異常検知
;; ;;前の動作終了時の力をref-fとして与えておｓ子から外れると落としたと判断
;; (defun check-while-still
;;     (rarm-ref-f larm-ref-f &key (checking-time 6) (sleep-time 100000) (thre-list (list 10 10)) (fin 3))

;;   (format t "~%~%check while still~%")
;;   (setq angle-pre (send *ri* :state :angle-vector))

;;   (setq flag 10)
;;   (dotimes (i (/ (* checking-time 1000000) sleep-time))
;;     (setq rarm-f  (low-pass-online "force" "absolute" :rarm :times 30 :sleep 100 :flag nil))
;;     (setq larm-f  (low-pass-online "force" "absolute" :larm :times 30 :sleep 100 :flag nil))

;;     ;;絶対値バージョン
;;     ;; (setq rarm-f-dif (abs (elt (v- rarm-ref-f rarm-f) 2)))
;;     ;; (setq larm-f-dif (abs (elt (v- larm-ref-f larm-f) 2)))
;;     (setq rarm-f-dif (elt (v- rarm-ref-f rarm-f) 2))
;;     (setq larm-f-dif (elt (v- larm-ref-f larm-f) 2))
      
;;     (setq drop (list nil nil))
;;     (format t "rarm-f-dif      = ~A~%" rarm-f-dif)
;;     (format t "larm-f-dif      = ~A~%" larm-f-dif)

;;     ;;drop判定
;;     (if (> rarm-f-dif (* +1 (elt thre-list 0)))
;;         (setq drop (list-insert t 0 drop)))
;;     (if (>  larm-f-dif (* +1 (elt thre-list 1)))
;;         (setq drop (list-insert t 1 drop)))
    
;;     (if (or (elt drop 0) (elt drop 1))  
;;         (progn
;;           (print "drop")
;;           (send *ri* :stop-motion)
;;           (print "stop motion")
;;           (if (and (elt drop 0) (elt drop 1))
;;               (progn
;;                 (print "drop both arm")
;;                 (setq flag 0)
;;                 (return 0))
;;             (progn
;;               (cond ((elt drop 0)
;;                      (progn
;;                        (print "drop rarm")
;;                        (setq flag 1)
;;                        (return 1)))
;;                     ((elt drop 1)
;;                      (progn
;;                        (print "drop larm")
;;                        (setq flag 2)
;;                        (return 2))))))))

;;     (setq angle-pre (send *ri* :state :angle-vector))
;;     (unix:usleep sleep-time)
;;     )
;;   (format t "flag = ~A~%" flag)
;;   flag

;;   )
