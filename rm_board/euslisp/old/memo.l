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
(defun get-half-plane-save(obj &key (offset 500))
  (setq origin-obj (copy-object obj))
  (setq origin-vertices (send (car (send origin-obj :faces)) :vertices))
  (setq com (send origin-obj :centroid))
  (setq len-lis (length origin-vertices))
  (setq back (mapcar #'(lambda (l) (progn (if (> (elt l 0) (+ (elt com 0) offset)) l))) origin-vertices))
  (setq back-vertices (intersection back origin-vertices :test #'equal))
  (setq i 0)
  (setq back-vertices (flatten (mapcar #'(lambda (l) (progn (setq tmp (list (v+ #f(0 0 0) (scale 0.5 (v+ (elt back-vertices i) l))) l)) (setq i (+ i 1)) (return tmp))) (cdr back-vertices))))
  (setq back-obj (make-prism (mapcar #'(lambda (l) (v+ #f(0 0 0) l)) back-vertices) 10))
  (format t "back vertices = ~A~%" back-vertices)
  ;; (setq back-obj (make-prism (cdr back-vertices) 20))
  (send back-obj :set-color (float-vector 0 1 0))
  ;; (send origin-obj :set-color (float-vector 0 0 1))  
  (format t "origin-obj = ~A~%" origin-obj)
  (format t "back-obj = ~A~%" back-obj)
  (objects (list back-obj origin-obj))
  (setq front-obj (body- origin-obj back-obj))
  (send front-obj :set-color (float-vector 1 0 0))
  (objects (list front-obj origin-obj))
  )

;;とりあえずこれで行く
(defun get-half-plane(obj &key (cut-rate 0.5))
  (setq origin-obj (copy-object obj))
  (setq origin-vertices (send (car (send origin-obj :faces)) :vertices))
  (setq com (send origin-obj :centroid))
  (format t "com = ~A~%" com)
  (setq len-lis (length origin-vertices))
  (setq cub (make-cube (* (/ 1.0 cut-rate) (abs (elt com 0))) 5000 5000))
  (send cub :translate (scale 2.0 com))
  (setq sub (body- origin-obj cub))
  (send sub :set-color (float-vector 1 0 0))  
  (objects (list origin-obj sub ))
  sub)
(defun get-half-plane(plane &key (offset 500))
  (setq origin-face plane)
  (setq vertices-list (send origin-face :vertices))
  (setq com (send origin-face :centroid))
  ;; (setq sum 0)
  (setq len-lis (length vertices-list))
  ;; (dotimes (i len-lis) (progn  (setq sum (+ sum (elt (elt vertices-list i) 0)))))
  ;; (setq ave (/ sum len-lis))
  ;; (setq new-vertices-list (mapcar #'(lambda (l) (progn (if (< (elt l 0) (+ ave offset)) l))) vertices-list))
  (setq new-vertices-list (mapcar #'(lambda (l) (progn (if (< (elt l 0) (+ (elt com 0) offset)) l))) vertices-list))
  (setq new-vertices (intersection new-vertices-list vertices-list :test #'equal))
  ;; (setq latest-vertices (list-insert com len-lis new-vertices))
  (setq norm-list (mapcar #'(lambda (l) (progn (norm (v- l com)))) new-vertices))
  (setq latest-vertices (list-insert com (- (- len-lis (length (member (car (quick-sort norm-list)) new-vertices))) 1) new-vertices))
  ;; (setq latest-vertices  new-vertices)
  (setq face-arm (make-prism latest-vertices 10))
  (send face-arm :set-color (float-vector 1 0 0))
  (objects (list face-arm origin-face))
  (format t "com = ~A~%" com)
  (format t "origin vertices = ~A~%" vertices-list)
  (format t "new vertices = ~A~%" latest-vertices)
  )

(defun get-half-plane(plane &key (offset 500))
  (setq origin-face plane)
  (setq vertices-list (send origin-face :vertices))
  (setq com (send origin-face :centroid))
  (setq len-lis (length vertices-list))
  (setq new-vertices-list (mapcar #'(lambda (l) (progn (if (< (elt l 0) (+ (elt com 0) offset)) l))) vertices-list))
  (setq new-vertices (intersection new-vertices-list vertices-list :test #'equal))
  ;; (setq latest-vertices (list-insert com len-lis new-vertices))

  ;; (setq norm-list (mapcar #'(lambda (l) (progn (norm (v- l com)))) new-vertices))
  ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (car (quick-sort norm-list)) new-vertices))) 1) new-vertices))
  ;; (setq latest-vertices  new-vertices)
  (setq i 0)
  ;; (setq shift-list (list-insert (car new-vertices) len-lis (cdr new-vertices)))
  (setq latest-vertices (flatten (mapcar #'(lambda (l) (progn (setq tmp (list (scale 0.5 (v+ (elt new-vertices i) l)) l)) (setq i (+ i 1)) (return tmp))) (cdr new-vertices))))
  ;; (setq norm-list (mapcar #'(lambda (l) (progn (norm (v- l com)))) new-vertices))
  (setq i 0)
  ;; (setq norm-list (mapcar #'(lambda (l) (progn (setq tmp (abs (elt (v- (elt latest-vertices i) l) 1))) (setq i (+ i 1)) (return tmp))) ;; (cdr latest-vertices)
  ;;                         (list-insert (car latest-vertices) 300 (cdr latest-vertices))))
  ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (last (quick-sort norm-list)) latest-vertices))) -1) latest-vertices))
  ;; (setq latest-vertices (list-insert com (- (- len-lis (length (member (car (quick-sort norm-list)) new-vertices))) 1) new-vertices))
  ;; (setq latest-vertices (list-insert (car latest-vertices) 300 latest-vertices))
  (setq face-arm (make-prism (cdr latest-vertices) 10))
  (send face-arm :set-color (float-vector 1 0 0))
  (objects (list face-arm origin-face))
  (format t "com = ~A~%" com)
  (format t "origin vertices = ~A~%" vertices-list)
  (format t "latest vertices = ~A~%" latest-vertices)
  )
(defun ttt()
  (setq back (mapcar #'(lambda (l) (progn (if (> (elt l 0) (+ (elt com 0) offset)) l))) origin-vertices))
  (setq back-vertices (intersection back origin-vertices :test #'equal))
  (setq i 0)
  (setq back-vertices (flatten (mapcar #'(lambda (l) (progn (setq tmp (list (v+ #f(0 0 0) (scale 0.5 (v+ (elt back-vertices i) l))) l)) (setq i (+ i 1)) (return tmp))) (cdr back-vertices))))
  (setq back-obj (make-prism (mapcar #'(lambda (l) (v+ #f(0 0 0) l)) back-vertices) 10))
  (format t "back vertices = ~A~%" back-vertices)
  ;; (setq back-obj (make-prism (cdr back-vertices) 20))
  (send back-obj :set-color (float-vector 0 1 0))
  ;; (send origin-obj :set-color (float-vector 0 0 1))  
  (format t "origin-obj = ~A~%" origin-obj)
  (format t "back-obj = ~A~%" back-obj)
  (objects (list back-obj origin-obj))
  (setq front-obj (body- origin-obj back-obj))
  (send front-obj :set-color (float-vector 1 0 0))
  (objects (list front-obj origin-obj))
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


;;interactive marker calib 1208
;; <launch>

;; <node name="marker_6dof"
;; pkg="jsk_interactive_marker" type="marker_6dof" >
;; <rosparam>
;; frame_id: HEAD_LINK1
;; object_type: none
;; tf_frame: rs_l515_link
;; publish_tf: true
;; initial_x: 0.140
;; initial_y: 0.0
;; initial_z: -0.01
;; initial_orientation: [0, 0.398, 0.000, 0.917]
;; </rosparam>
;; </node>

;; </launch>
;; rosrun tf tf_echo /HEAD_LINK1 /rs_l515_link
-13508

#f(0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 0.0 0.0 30.0 0.0 55.0 -20.0 -15.0 -100.0 -25.0 0.0 -45.0 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)
#<cascaded-coords #X81f6b38 :lleg-end-coords  8.631e-13 100.0 1.705e-13 / 6.939e-18 -2.776e-16 -7.704e-34>

;;1215
rtprint -t 0.1 jaxonred:15005/rfu.rtc:ref_lhsensorOut
rtprintはctr-dで中止すること（hrpsysがとまってしまう
rtcat jaxonred:15005/rfu.rtc:ref_lhsensorOut -l


stp=hcf.abc_svc.getStabilizerParam()
stp.joint_servo_control_parameters
[OpenHRP.AutoBalancerService.JointServoControlParameter(support_pgain=[5.0, 10.0, 10.0, 5.0, 0.09999999999999998, 0.09999999999999998], support_dgain=[10.0, 20.0, 20.0, 10.0, 10.0, 10.0], landing_pgain=[5.0, 1.0, 1.0, 1.0, 0.09999999999999998, 0.09999999999999998], landing_dgain=[10.0, 10.0, 10.0, 10.0, 5.0, 5.0], swing_pgain=[5.0, 30.0, 20.0, 10.0, 5.0, 5.0], swing_dgain=[10.0, 30.0, 20.0, 20.0, 10.0, 10.0]),
OpenHRP.AutoBalancerService.JointServoControlParameter(support_pgain=[5.0, 10.0, 10.0, 5.0, 0.09999999999999998, 0.09999999999999998], support_dgain=[10.0, 20.0, 20.0, 10.0, 10.0, 10.0], landing_pgain=[5.0, 1.0, 1.0, 1.0, 0.09999999999999998, 0.09999999999999998], landing_dgain=[10.0, 10.0, 10.0, 10.0, 5.0, 5.0], swing_pgain=[5.0, 30.0, 20.0, 10.0, 5.0, 5.0], swing_dgain=[10.0, 30.0, 20.0, 20.0, 10.0, 10.0]),
OpenHRP.AutoBalancerService.JointServoControlParameter(support_pgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], support_dgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], landing_pgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], landing_dgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], swing_pgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], swing_dgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]),
OpenHRP.AutoBalancerService.JointServoControlParameter(support_pgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], support_dgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], landing_pgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], landing_dgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], swing_pgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0], swing_dgain=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0])]


only-step-fail
send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :fix-contact? nil
init-lcoords = #<coordinates #X7f234a494420  5.684e-14 100.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
init-rcoords = #<coordinates #X7f234a4942b8  5.684e-14 -100.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-lcoords = #<coordinates #X7f231d5848e0  100.0 100.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-rcoords = #<coordinates #X7f231d584718  100.0 -100.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
Do you move foot?(Y or N): y
Do you move real machine?(Y or N): n
nil
75.B11-irteusgl$ send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0 -100 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(0 100 0)) :fix-contact? nil
init-lcoords = #<coordinates #X7f2316900778  100.0 100.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
init-rcoords = #<coordinates #X7f2316978ee0  100.0 -100.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-lcoords = #<coordinates #X7f2316978d30  100.0 200.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-rcoords = #<coordinates #X7f23169755b0  100.0 -200.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
Do you move foot?(Y or N): y
Do you move real machine?(Y or N): n
nil
76.B11-irteusgl$ send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :fix-contact? nil
init-lcoords = #<coordinates #X7f23160980c0  100.0 5.684e-14 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
init-rcoords = #<coordinates #X7f231609c8a8  100.0 -200.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-lcoords = #<coordinates #X7f231609c6f8  200.0 5.684e-14 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-rcoords = #<coordinates #X7f231609c548  200.0 -200.0 -5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>




progn (send *ri* :start-log) (send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :fix-contact? nil) (send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0 -100 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(0 100 0)) :fix-contact? nil) (send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :fix-contact? nil)
init-lcoords = #<coordinates #X7f2306588ec8  1.319e-60 100.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
init-rcoords = #<coordinates #X7f2306588d48  1.319e-60 -100.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-lcoords = #<coordinates #X7f2306351570  100.0 100.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-rcoords = #<coordinates #X7f23063513c0  100.0 -100.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
Do you move foot?(Y or N): y
Do you move real machine?(Y or N): n
init-lcoords = #<coordinates #X13a8af18  100.0 100.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
init-rcoords = #<coordinates #X18bc0b48  100.0 -100.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-lcoords = #<coordinates #X18bbfe88  100.0 200.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-rcoords = #<coordinates #X18bbfcd8  100.0 -200.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
Do you move foot?(Y or N): y
Do you move real machine?(Y or N): n
init-lcoords = #<coordinates #X7f233cfa0ba0  100.0 5.684e-14 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
init-rcoords = #<coordinates #X7f233cfa0a20  100.0 -200.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-lcoords = #<coordinates #X7f233cfa0870  200.0 5.684e-14 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-rcoords = #<coordinates #X7f233cf9ffd0  200.0 -200.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
Do you move foot?(Y or N): y
Do you move real machine?(Y or N): n
nil
94.B13-irteusgl$ progn (send *ri* :start-log) (send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :fix-contact? nil) (send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0 -100 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(0 100 0)) :fix-contact? nil) (send-pose-and-move-on-foot :rleg-coords-new (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :lleg-coords-new (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(100 0 0)) :fix-contact? nil)
init-lcoords = #<coordinates #X7f2312015470  200.0 5.684e-14 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
init-rcoords = #<coordinates #X7f231330f578  200.0 -200.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-lcoords = #<coordinates #X7f231330b720  300.0 5.684e-14 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-rcoords = #<coordinates #X7f231330b558  300.0 -200.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
Do you move foot?(Y or N): y
Do you move real machine?(Y or N): n
init-lcoords = #<coordinates #X18bec1b0  300.0 8.527e-14 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
init-rcoords = #<coordinates #X18bec030  300.0 -200.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-lcoords = #<coordinates #X18bebe80  300.0 100.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
next-rcoords = #<coordinates #X18beb5b0  300.0 -300.0 5.684e-14 / 7.347e-40 9.420e-63 -2.564e-23>
Do you move foot?(Y or N): y
Do you move real machine?(Y or N):   C-c C-c



1230

init-lcoords = #<coordinates #X26d70ae8  1.628e-13 100.0 1.705e-13 / -4.163e-17 5.551e-17 -5.204e-18>
init-rcoords = #<coordinates #X26d70968  1.628e-13 -100.0 1.705e-13 / -4.163e-17 5.551e-17 -5.204e-18>
next-lcoords = #<coordinates #X27cf8708  156.984 89.725 -0.007 / 0.025 -1.202e-06 6.114e-06>
next-rcoords = #<coordinates #X27c36310  156.986 -176.993 -0.012 / -0.014 -1.235e-07 5.902e-06>

init-lcoords = #<coordinates #X12a9e940  156.919 89.725 -0.0 / 0.025 1.185e-06 6.257e-06>
init-rcoords = #<coordinates #X12a9e550  156.927 -176.97 -0.012 / -0.014 4.107e-06 2.536e-06>
next-lcoords = #<coordinates #X7f02be5d5e20  284.179 157.358 -0.003 / 0.015 -1.071e-06 -1.888e-06>
next-rcoords = #<coordinates #X7f02be5d5ca0  290.357 -184.275 -0.015 / 0.011 -3.037e-07 -5.031e-07
failstep while heave-with-sensor
init-lcoords = #<coordinates #X27c86fe0  284.179 157.358 -0.003 / 0.015 -1.071e-06 -1.888e-06>
init-rcoords = #<coordinates #X27c85c00  290.357 -184.275 -0.015 / 0.011 -3.037e-07 -5.031e-07>
next-lcoords = #<coordinates #X7f02bd55bca0  448.622 131.89 0.035 / -0.09 -4.764e-06 2.160e-05>
next-rcoords = #<coordinates #X7f02bd55bb20  395.97 -250.115 -0.002 / -0.075 -5.421e-06 2.873e


0102
#f(0.897233 -5.17316 -30.3655 70.966 -32.5286 3.39279 0.817664 2.15006 -29.2349 69.2021 -31.8935 -3.86937 -0.084729 13.3282 0.165979 0.113276 26.0 37.0 23.572 -24.405 5.82802 -61.7925 45.8089 -15.8428 9.01074 -37.0 22.7147 22.8779 -5.17243 -54.4999 -36.9803 22.1565 19.5136)
25.B2-irteusgl$ send *coords* :get-lleg-coords
#<coordinates #X7f541de78528  104.745 109.342 -0.009 / -0.007 -3.460e-07 -1.236e-06>
26.B2-irteusgl$ send *coords* :get-rleg-coords
#<coordinates #X7f541de783a8  100.497 -176.101 -0.009 / -0.024 -2.481e-07 1.708e-06>



;;calib
absolute
com-list =(#f(0.089906 6.881544e-05 0.071121) #f(0.090729 -0.000351 0.080593) #f(0.089672 -1.886754e-05 0.071464) #f(0.088781 -0.000354 0.07266) #f(0.087193 -0.001668 0.075643) #f(0.090167 -0.001309 0.071161) #f(0.088462 -0.003424 0.073298) #f(0.089761 -0.000719 0.074825) #f(0.086913 -0.003183 0.068969) #f(0.083521 -0.003543 0.070206) #f(0.0 0.0 0.0))
mass-list =(#f(2.02723 1.3927 2.40293) #f(2.04367 1.33752 2.4569) #f(2.12773 1.41488 2.42028) #f(2.0097 1.35053 2.47296) #f(1.96403 1.34432 2.47074) #f(1.99949 1.36037 2.38475) #f(1.92263 1.24201 2.47637) #f(2.02394 1.35836 2.40218) #f(1.93691 1.27281 2.47826) #f(1.8055 1.27159 2.59443) #f(0.0 0.0 0.0))
(#f(0.088511 -0.00145 0.072994) #f(1.98608 1.33451 2.45598))

remove force absolute
com-list =(#f(0.041679 0.03523 0.046796) #f(0.040143 0.034032 0.051252) #f(0.095908 0.010859 0.078308) #f(0.025326 0.049045 0.038997) #f(0.107198 0.001343 0.076713) #f(0.106116 0.012864 0.070011) #f(0.192328 -0.035824 0.12114) #f(0.171651 -0.009946 0.108096) #f(0.291337 -0.065258 0.18873) #f(0.326923 -0.055464 0.164123) #f(0.0 0.0 0.0))
mass-list =(#f(-0.274154 0.241142 0.0203) #f(-0.275922 0.220404 0.026107) #f(-0.406645 0.208342 0.16205) #f(-0.235608 0.371815 0.029601) #f(-0.411607 0.17431 0.173652) #f(-0.578784 0.22189 0.245043) #f(-0.533521 0.200647 0.250628) #f(-0.628046 0.165888 0.348307) #f(-0.609189 0.173198 0.276837) #f(-0.506541 0.180416 0.352634) #f(0.0 0.0 0.0))
(#f(0.139861 -0.002312 0.094417) #f(-0.446002 0.215805 0.188516))

absolute angle-vector更新version
com-list =(#f(0.00793 0.053367 0.028454) #f(0.010305 0.057964 0.032089) #f(0.006714 0.056347 0.0294) #f(0.013823 0.053675 0.033535) #f(0.010525 0.056067 0.029556) #f(0.012764 0.05696 0.03358) #f(0.037645 0.050635 0.050098) #f(0.029947 0.051336 0.043869) #f(0.033127 0.049766 0.045611) #f(0.041395 0.044845 0.052441) #f(0.0 0.0 0.0))
mass-list =(#f(-0.329179 -0.134816 -0.147402) #f(-0.302231 -0.116899 -0.119126) #f(-0.344866 -0.202434 -0.093652) #f(-0.297808 -0.091941 -0.088238) #f(-0.321679 -0.126318 0.017333) #f(-0.311944 -0.10116 -0.047388) #f(-0.482491 0.001963 0.080611) #f(-0.452596 -0.006185 -0.009765) #f(-0.44332 -0.024744 0.058748) #f(-0.458108 0.144075 0.014949) #f(0.0 0.0 0.0))
(#f(0.020417 0.053096 0.037863) #f(-0.374422 -0.065846 -0.033393))


;;wood com 45くらい
"force vector through filter and average = #f(0.079616 4.22854 -9.13749)"
"moment vector through filter and average = #f(-0.229901 3.48579 1.79101)"
coord1     = #<coordinates #X13816998  282.706 -341.918 954.342 / 0.055 0.338 -0.35>
f1 (world) = #f(3.67565 9.89325 -11.3402)
f1(coord1) = #f(7.73625 12.2791 -5.41863)
f-matrix1 = #2f((0.0 5.41863 12.2791) (-5.41863 0.0 -7.73625) (-12.2791 7.73625 0.0))

coord2     = #<coordinates #X1222cdb8  156.063 -525.102 836.471 / -0.33 0.739 -0.85>
f2 (world) = #f(0.079616 4.22854 -9.13749)
f2(coord2) = #f(5.19678 8.387 -2.00806)
f2(coord1) = #f(5.19678 8.387 -2.00806)
f-matrix2 = #2f((0.0 2.00806 8.387) (-2.00806 0.0 -5.19678) (-8.387 5.19678 0.0))

"moment  = #f(-2.35888 2.43008 2.99942 -2.20314 1.81447 2.6954)"
"f-matrix = #2f((0.0 5.41863 12.2791) (-5.41863 0.0 -7.73625) (-12.2791 7.73625 0.0) (0.0 2.00806 8.387) (-2.00806 0.0 -5.19678) (-8.387 5.19678 0.0))"
f-inverse-matrix = #2f((0.176598 -0.123117 -0.026862 -0.273026 0.159919 -0.03865) (0.297617 -0.171096 0.037193 -0.429663 0.264496 -0.007239) (-0.070724 0.042131 -0.005503 0.189969 -0.115667 0.00853))
com=#f(0.008825 -0.40075 0.352702) with pseudo-inverse
com norm = 0.533926
com-world =#f(0.177646 -0.24627 0.43917) with pseudo-inverse
com-local norm = 0.533926
com-local-vector subtract  offset=#f(-0.091175 -0.40075 0.352702)
com-local-vector subtract  offset norm =0.541583
(#f(0.008825 -0.40075 0.352702) #f(1.87763 7.06089 -10.2389) #f(-0.091175 -0.40075 0.352702))

(progn
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :move-gripper :larm 0.0 :wait t)
  (send *ri* :move-gripper :rarm 0.0 :wait t)
  (setq lleg-coords-new (make-coords :pos #f(48.356 245.005 -0.008) :rpy #f(0.022 4.965e-06 1.208e-05)))
  (setq rleg-coords-new (make-coords :pos #f(50 -249 -0.009) :rpy #f(-0.024 -2.481e-07 1.708e-06)))
  (setq zmp-vector (send *ri*  :zmp-vector)) 
  (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  (send *ri* :set-foot-steps
        (list
         (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
         (make-coords :coords  lleg-coords-new :name :lleg)
         (make-coords :coords  rleg-coords-new :name :rleg)
         ))
  )


;;emergency check pose
lleg #<cascaded-coords #X27681490 :lleg-end-coords  156.985 261.812 -0.012 / 0.014 1.315e-06 -9.876e-07>
rleg #<cascaded-coords #X274e19f8 :rleg-end-coords  156.984 1.812 -0.008 / -0.009 6.967e-07 -3.684e-06>
#f(-4.30965 2.78768 -46.61 64.8649 -19.4957 -0.259879 -2.8812 7.6575 -48.3618 65.4802 -18.4271 -5.15996 -1.76532 16.1927 -0.218483 -1.86632 10.0 37.0 -10.5421 -37.7312 11.7252 -48.9479 32.3991 -4.75814 18.1441 -37.0 -4.73756 49.4529 -5.47873 -48.3535 -34.726 -0.928479 3.70811)

pos-shift 500
#f(-4.66667 3.6799 -33.716 88.2643 -42.3469 -1.78091 -4.84388 8.62064 -35.6104 89.4537 -41.5584 -6.6467 -4.36627 25.0042 2.294 -0.543132 26.0 32.2141 21.1391 -23.111 0.368342 -39.3486 30.9508 -36.2867 -12.749 -36.2321 35.297 43.5624 9.46021 -31.1129 -20.1255 32.5158 -27.2312)
#<coordinates #X7ff8e1384510  156.996 85.072 -0.017 / 0.01 2.148e-07 4.530e-06>
#<coordinates #X7ff8e13950a0  156.998 -164.413 -0.02 / -0.006 6.047e-07 4.649e-06>

pos-shift 600
#f(4.12189 -8.41655 -40.3045 98.3299 -44.6068 6.71473 3.61035 -2.59642 -38.5497 97.5438 -45.6816 0.934533 5.11451 30.0 -3.68961 0.437587 26.0 33.1104 38.933 -53.4726 -14.7828 -28.6262 12.2443 -27.9315 -38.7648 -33.0899 27.6177 31.9098 8.8433 -33.7855 -18.1627 34.5063 -29.1881)
ver2
#f(-6.3788 8.02649 -39.2384 99.8233 -46.7566 -4.85518 -5.74145 10.1075 -42.2655 102.317 -46.2114 -6.72525 -5.29927 30.0 4.38191 -0.560371 26.0 32.865 24.7126 -30.109 -7.65283 -33.9454 18.416 -33.8113 -29.447 -33.4332 39.9734 57.3104 15.0845 -25.6932 -10.9549 27.0733 -39.1223)

;;tilted pose
reach-auto :send? nil :adjust? nil :fix-contact? nil :pos-shift 280 :ik2? nil :ik3? nil


;;
"force vector through filter and average = #f(-15.225 -9.00004 7.33008)"
"force vector through filter and average = #f(-4.34633 36.5878 -47.9027)"
init-lcoords = #<coordinates #X7f56bf93e8f0  2.849e-14 8.105e-15 1.421e-14 / 1.384e-17 -1.084e-18 2.776e-17>
init-rcoords = #<coordinates #X7f56bf93e098  16.215 -370.183 -0.002 / -0.041 -5.999e-07 4.696e-06>
next-lcoords = #<coordinates #X7f57852c1b08  190.253 62.217 -0.022 / -0.072 1.139e-07 2.250e-06>
next-rcoords = #<coordinates #X7f56bdfeaff8  204.87 -446.04 -0.019 / 0.027 7.562e-07 -1.726e-06>


;;0110
#f(-12.3487 -8.99059 -56.3237 93.5956 -39.3806 9.43667 10.9637 5.39824 -55.41 94.9267 -41.6176 -5.79678 2.89957 30.0 -2.67419 7.5447 -5.0 37.0 20.3756 -41.7093 -12.6859 -76.8937 46.9698 -15.2244 18.1064 -37.0 8.88096 28.7258 7.02275 -72.0492 -51.3543 18.9711 19.8344)

#f(-4.7808 -3.42782 -46.6759 88.2204 -42.9961 3.54708 0.202229 7.90216 -44.0985 87.2326 -44.6018 -7.90443 -5.62325 15.3424 6.59208 -11.9182 26.0 37.0 18.3776 -18.8251 -2.03448 -73.4568 52.5777 -16.0317 25.796 -37.0 27.4065 36.1098 7.72492 -70.4441 -42.8394 15.1899 10.9918)

;;
robot lcoords = #<coordinates #X7f6c394e9ff8  1.092e-17 -1.293e-14 0.0 / 3.465e-18 -1.820e-19 -1.388e-17>
robot rcoords = #<coordinates #X7f6c394e98a8  -59.735 -321.362 -0.007 / 0.021 9.093e-07 1.857e-06>
robot angle-vector = #f(0.845168 -4.76947 -39.7486 76.4653 -40.7972 4.69775 0.428295 5.99958 -45.3741 76.0349 -34.7505 -6.01463 -1.82797 7.89905 -5.87246 4.67899 1.83123 37.0 10.28 -27.6137 -0.426646 -64.7098 50.3253 -4.42444 15.1117 -37.0 23.5686 28.2448 6.40036 -65.7202 -40.0836 13.8851 15.4743)



#f(-1.704 -6.44234 -42.038 88.5287 -47.1159 6.46033 0.05511 5.10885 -45.2001 88.979 -44.4028 -5.10899 1.13229 11.6047 -7.23663 2.69917 0.824779 37.0 26.4666 -34.0665 -4.80595 -73.4077 43.7561 -8.55578 13.5856 -37.0 25.2936 19.536 4.02469 -77.5973 -48.1484 12.0398 25.3355)

;;111 12:41 ipad2
init-lcoords = #<coordinates #X7f6c52c91cf0  5.435e-14 4.446e-14 -1.421e-13 / 6.797e-17 2.195e-16 1.388e-17>
init-rcoords = #<coordinates #X7f6c52c91b88  -210.556 -375.72 0.021 / 0.319 -1.113e-05 -3.014e-06>
next-lcoords = #<coordinates #X7f6a586ef310  37.009 3.933 -0.024 / -0.035 -2.027e-06 1.375e-07>
next-rcoords = #<coordinates #X7f6a587035c0  -34.33 -367.14 0.063 / 0.139 -1.420e-06 -1.671e-05

;;0113
send *ri* :set-auto-balancer-param :default-zmp-offsets (list #f(20 20 0.0) #f(20 -20 0.0) #f(0.0 0.0 0.0) #f(0.0 0.0 0.0))
send (send (send *ri* :get-auto-balancer-param) :default_zmp_offsets) :data
;;debag zmp
(progn (send *ri* :start-log) (unix:sleep 5) (debag-func) (unix::sleep 10) (save-log :log-dir "matsuura/log"))


;;calib memo -28363
