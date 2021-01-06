#!/usr/bin/env roseus

(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(load "make_plane_for_opt.l")
(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

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
(defun get-target-pose(contact-wall arm)
  (let* ((idx-x 0)
        (idx-y 0)
        (longest-len 0)
        (vertices-list (get-min-val (send contact-wall :vertices) 20))
        (vertices-list-reverse (reverse vertices-list))
        (plane-normal (send (elt (send contact-wall :faces) 0) :normal)))
    (format t "plane normal = ~A~%" plane-normal)
  (dotimes (i (length vertices-list))
    (dotimes (j (- (length vertices-list) (+ 1 i)))
      (setq len-tmp (norm (v- (elt vertices-list i) (elt vertices-list-reverse j))))
      (if (< longest-len len-tmp)
          (progn
            (setq idx-x i)
            (setq idx-y j) ;;reverseされているがそのままいく
            (setq longest-len len-tmp)))))
  (setq longest-edge (make-line (elt vertices-list idx-x) (elt vertices-list-reverse idx-y)))
  (format t "longest-len = ~A~%" longest-len)
  (format t "vertice = ~A~%" (elt vertices-list idx-x))
  (format t "vertice = ~A~%" (elt vertices-list-reverse idx-y))  
  (setq normal-line (make-line #f(0 0 0) plane-normal))
  
  ;;手先の目標
  (setq sum #f(0 0 0))
  (dotimes (i (length vertices-list))
    (setq sum (v+ sum (elt vertices-list i))))
  (setq pos (scale (/ 1.0 (length vertices-list)) sum))
  (format t "mid-pos = ~A~%" pos)
  (setq y-vec  plane-normal)
  (setq z-vec (v- (elt vertices-list idx-x) (elt vertices-list-reverse idx-y)))
  (setq x-vec (v* y-vec z-vec))
  ;;x-vecは正面方向
  (if (< (elt x-vec 0) 0)
      (setq x-vec (scale -1 x-vec)))

  (if (equal arm "larm")
      (progn
        (setq pos (v+ pos #f(0 +320 0)))
        (if (< (elt y-vec 2) 0)
          (setq y-vec (scale -1 y-vec)))
        (if (> (elt z-vec 1) 0)
            (setq z-vec (scale -1 z-vec))))
    (progn
      (setq pos (v+ pos #f(0 -320 0)))
      (if (> (elt y-vec 2) 0)
        (setq y-vec (scale -1 y-vec)))
      (if (< (elt z-vec 1) 0)
          (setq z-vec (scale -1 z-vec)))))

  (setq x-vec (normalize-vector x-vec))
  (setq y-vec (normalize-vector y-vec))
  (setq z-vec (normalize-vector z-vec))
  ;; (format t "plane-normal = ~A~%" plane-normal)
  (format t "x-vec = ~A~%" x-vec)
  (format t "y-vec = ~A~%" y-vec)
  (format t "z-vec = ~A~%" z-vec)
  (setq rot (transpose
             (matrix x-vec y-vec z-vec)))
  (format t "pos = ~A~%" pos)
  (format t "rot = ~A~%" rot)
  (list pos rot)
  ))


(defun auto-set-reaching-targets(&key (larm-shift #f(-150 -50 0)) (rarm-shift #f(-150 50 0)))
  (setq cu (make_plane))
  (if (not cu)
      (progn
        (print "can not find any plane")
        (return-from auto-set-reaching-targets nil)))
  (setq obj-larm (copy-object cu))
  (setq obj-rarm (copy-object cu))
  (setq ret-larm (get-target-pose obj-larm "larm"))
  (setq ret-rarm (get-target-pose obj-rarm "rarm"))

  (setq coords1 (make-coords))
  (setq larm-coords (make-coords :pos (elt ret-larm 0) :rpy (elt (rpy-angle (elt ret-larm 1)) 0)))
  (setq rarm-coords (make-coords :pos (elt ret-rarm 0) :rpy (elt (rpy-angle (elt ret-rarm 1)) 0)))
  
  ;;少し回転
  (setq larm-coords (send larm-coords :rotate (deg2rad -20) :z))
  (setq rarm-coords (send rarm-coords :rotate (deg2rad 20) :z))

  (setq larm-coords (send larm-coords :translate larm-shift))
  (setq rarm-coords (send rarm-coords :translate rarm-shift))
  (format t "ik larm-coords = ~A~%" larm-coords)
  (format t "ik rarm-coords = ~A~%" rarm-coords)

  (objects (list cu coords1 larm-coords rarm-coords))
  (list larm-coords rarm-coords)
  )
