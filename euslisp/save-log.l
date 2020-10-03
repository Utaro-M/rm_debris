(defun save(&key (fname "emergency"))
  ;; (setq path "~/google-drive/JSK/log")
    (setq path "/home/utaro/google-drive/JSK/log")
                 
  (send *ri* :save-log (format nil "~A/~A" path name) :make-directory t)
)
