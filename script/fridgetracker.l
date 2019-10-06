(load "utils.l")
(setq *use-fetch* nil)
(if *use-fetch*
    (progn
      (load "package://fetcheus/fetch-interface.l")
      (fetch-init)
      )
    (progn
      (load "package://pr2eus/pr2-interface.l")
      (pr2-init)))
(setq *camera-frame* (if *use-fetch* 
                         "/head_camera_rgb_optical_frame"
                         "/head_mount_kinect_rgb_optical_frame"))

(ros::load-ros-manifest "roseus")
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
  (ros::advertise "/base_controller/command" geometry_msgs::Twist 1)
  (speak-jp "動きます")
  (loop 
    (print "hoge")

    (setq *msg* (instance geometry_msgs::Twist :init))
    (send *msg* :linear :x 0.2)
    (send *msg* :angular :z 0)
    (ros::publish "/base_controller/command" *msg*)
    (if *use-fetch*
      (unix:usleep 200000) )

    (ros::spin-once)
    (send robot :head_pan_joint 
          :joint-angle (compute-direction *co-fridge*))
    (let* ((angle-now (send robot :head_pan_joint :joint-angle))
           (angle-diff (- (compute-direction *co-fridge*) angle-now))
           (angle-cmd (+ angle-now (* 2 angle-diff))))
      (send robot :head_pan_joint :joint-angle angle-cmd)
      (transmit robot :wait? nil :duration 2000))))
(track *pr2*)




#|
(send *fetch* :head_pan_joint :joint-angle -90)
;(defun lookaround (robot)

(set-home-posture *fetch*)
(send *fetch* :head_pan_joint :joint-angle 60)
(transmit *fetch* :wait? t :duration 1000)
(send *fetch* :head_pan_joint :joint-angle -60)
(transmit *fetch* :wait? nil :duration 1000)

(ros::spin-once)
|#





