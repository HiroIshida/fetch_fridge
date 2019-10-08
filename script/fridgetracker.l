(load "load-common.l")
(ros::roseus "tracker" :anonymous t)

(setq *tfl* (instance ros::transform-listener :init))

(setq *co-fridge* nil)
(ros::subscribe "/fridge_pose" geometry_msgs::PoseStamped
                #'(lambda (msg) 
                    (let* ((pose (send msg :pose))
                           (lt (send *tfl* :lookup-transform
                                     "base_link" *camera-frame*
                                     (ros::time 0))))
                      (setq *co-fridge* (send lt :transform (ros::tf-pose->coords pose))))))

#|
(defun convert-vector (vec)
  (let* ((lt (send *tfl* :lookup-transform "base_link" "map" (ros::time 0)))
         (co-src (make-coords)))
    (send co-src :translate vec)
    (send (send lt :transform co-src) :worldpos)))
|#

(defun compute-direction (co)
  (let* ((vec (send co :worldpos))
         (x (aref vec 0))
         (y (aref vec 1))
         (deg (rad2deg (atan2 y x))))
    deg))

(defun compute-bodyangle-to-fridge (co)
  (let* ((x-vec (transform (send *co-fridge* :worldrot) #f(1 0 0)))
         (theta (atan (aref x-vec 1) (aref x-vec 0))))
    theta))
#|
(compute-bodyangle-to-fridge *co-fridge*)
|#






(defun lookaround (robot)
  (speak-wait "はじめます")
  (send robot :head_pan_joint :joint-angle 50)
  (transmit robot :wait? t :duration 1000)
  (let ((angle 50)(av-tmp *home-av*))
    (loop
      (ros::spin-once)
      (setq angle (- angle 20))
      (send robot :head_pan_joint :joint-angle angle)
      (transmit robot :wait? t :duration 1000)
      (if (< angle -50)
          (return))
      )))

(defun tracking (robot)
  (loop
    (ros::spin-once)
    (send robot :head_pan_joint :joint-angle)))

;; init
(set-home-posture *fetch*)
(setq *home-av* (send *fetch* :angle-vector))
(transmit *fetch* :wait? t :duration 2000)

(lookaround *fetch*)

;; find picture
(let ((av *home-av*))
  (loop 
    ;(speak-wait "あ")
    (print "hoge")
    (ros::spin-once)
    (send *fetch* :head_pan_joint :joint-angle (compute-direction *co-fridge*))
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000 :head-controller)
    (unix:usleep 1000000)
    )
  )



#|
(unix:sleep 3)
(ros::spin-once)
(print *co-fridge*)
(loop
  (print "hoge")
  (setq *angle* (compute-direction *co-fridge*))
  (print *angle*)
  (ros::spin-once)
  (setf (aref *home-av* 8) *angle*)
  (send *ri* :angle-vector-raw *home-av* 1000 :head-controller)
  (send *ri* :wait-interpolation)
  )
(send *fetch* :head_pan_joint :joint-angle 50)
(send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
|#

#|
(loop 
  (ros::spin-once)
  (print (compute-direction *co-fridge*))
  (send *pr2* :head_pan_joint :joint-angle (compute-direction *co-fridge*))
  (transmit *pr2* :wait? t :duration 2000)
  )
|#


;; 0.2 -0.06
#|
(defun track (robot)
  (speak-jp "動きます")
  (let ((angle-diff-pre 0))
    (loop 
      (print "hoge")
      (if *use-fetch*
        (unix:usleep 200000))
      (ros::spin-once)
      (let* ((angle-now (send robot :head_pan_joint :joint-angle))
             (angle-diff (- (compute-direction *co-fridge*) angle-now))
             (angle-diff-diff (- angle-diff angle-diff-pre)))
        (let* ((p-gain 0.0)
               (d-gain 0.0)
               ;(angle-cmd (+ angle-now (* p-gain angle-diff) (* d-gain angle-diff-diff)))
               (angle-cmd (compute-direction *co-fridge*)))
          (print (/ (abs (* p-gain angle-diff)) (abs (* d-gain angle-diff-diff))))
          (setq angle-diff-pre angle-diff)
          (send robot :head_pan_joint :joint-angle angle-cmd)
          (transmit robot :wait? nil :duration 1000))))))
|#
;(track *pr2*)




