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

(setq *range-min* nil)
(ros::subscribe "/basescan_range_min" std_msgs::Float32
                #'(lambda (msg) 
                    (setq *range-min* (send msg :data)))) 

(defun compute-direction (co)
  (let* ((pos (send co :worldpos))
         (x (aref pos 0))
         (y (aref pos 1))
         (deg (+ -90 (rad2deg (atan2 x y)))))
    (- deg)))


(defun track (robot)
  (speak-jp "動きます")
  (loop 
    (print "hoge")
    (if *use-fetch*
      (unix:usleep 200000))

    (ros::spin-once)
    (send robot :head_pan_joint 
          :joint-angle (compute-direction *co-fridge*))
    (let* ((angle-now (send robot :head_pan_joint :joint-angle))
           (angle-diff (- (compute-direction *co-fridge*) angle-now)))
      (let* ((p-gain 5.0)
             (angle-cmd (+ angle-now (* p-gain angle-diff))))
        (send robot :head_pan_joint :joint-angle angle-cmd)
        (transmit robot :wait? nil :duration 2000)))))
(track *pr2*)




