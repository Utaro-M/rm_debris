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
  (format t "left = ~A~%" left)
  (format t "right = ~A~%" right)
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
  (format t "left = ~A~%" left)
  (format t "right = ~A~%" right)
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
