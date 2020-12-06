#!/usr/bin/env roseus
(ros::load-ros-manifest "rm_debris_vision")
(ros::roseus "make_plane_for_ik_client")
(ros::wait-for-service "make_plane")

(defun make_plane()

  (let ((req (instance rm_debris_vision::PolygonRequest :init)))
    (setq res (ros::service-call "make_plane" req t))
    (if (not (send res :pts :polygon :points))
    (progn ;; (print "can not find any plane")
      (return-from make_plane nil)))
;; (while (not (send res :pts :polygon :points))
;;   (print "can not find any plane")
;;   (setq res (ros::service-call "make_plane" req t)))
    (format t "res =~A~%" res )

    (setq coords (make-coords))
    (setq com-box (make-cube 100 100 100))
    (send com-box :set-color #f(1 0 0))

    (setq vertices (mapcar #'(lambda (vertice) (scale 1000 (float-vector (send vertice :x) (send vertice :y) (send vertice :z)))) (send res :pts :polygon :points)))
    (setq my-plane (make-prism vertices 10))

    ;; (setq my-plane (body+ my-plane1 my-plane2))
    (setq com (scale 1 (send my-plane :centroid)))
    ;; (print com)
    (send com-box :translate com)
    (objects (list my-plane com-box coords))
    (send com-box :translate (scale -1 com))
    my-plane
    ))
;; (defun make_plane()

;;   (let ((req1 (instance rm_debris_vision::PolygonRequest :init))
;;         (req2 (instance rm_debris_vision::PolygonRequest :init)))
;;     (setq res (list (ros::service-call "make_plane" req1 t) (ros::service-call "make_plane" req2 t)))
;; (if (or (not (send (elt res 0) :pts :polygon :points)) (not (send (elt res 1) :pts :polygon :points)))
;;     (progn ;; (print "can not find any plane")
;;            (return-from make_plane nil)))
;; ;; (while (not (send res :pts :polygon :points))
;; ;;   (print "can not find any plane")
;; ;;   (setq res (ros::service-call "make_plane" req t)))
;; (format t "res =~A~%" res )

;; (setq coords (make-coords))
;; (setq com-box (make-cube 100 100 100))
;; (send com-box :set-color #f(1 0 0))

;; (setq vertices1 (mapcar #'(lambda (vertice) (scale 1000 (float-vector (send vertice :x) (send vertice :y) (send vertice :z)))) (send (elt res 0) :pts :polygon :points)))
;; (setq vertices2 (mapcar #'(lambda (vertice) (scale 1000 (float-vector (send vertice :x) (send vertice :y) (send vertice :z)))) (send (elt res 1) :pts :polygon :points)))

;; (setq my-plane1 (make-prism vertices1 10))
;; (setq my-plane2 (make-prism vertices2 10))
;; ;; (setq my-plane (body+ my-plane1 my-plane2))
;; (setq com (scale 1 (send my-plane :centroid)))
;; ;; (print com)
;; (send com-box :translate com)
;; (objects (list my-plane com-box coords))
;; (send com-box :translate (scale -1 com))
;;  my-plane
;; ))
