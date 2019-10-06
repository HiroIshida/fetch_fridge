(load "utils.l")
(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(ros::load-ros-manifest "roseus")
(ros::roseus "test" :anonymous t)

(setq *tfl* (instance ros::transform-listener :init))

(setq *co-fridge* nil)
(ros::subscribe "/fridge_pose" geometry_msgs::PoseStamped
                #'(lambda (msg) 
                    (let* ((pose (send msg :pose))
                           (lt (send *tfl* :lookup-transform
                                     "base_link" "/head_camera_rgb_optical_frame"
                                     (ros::time 0))))
                      (setq *co-fridge* (send lt :transform (ros::tf-pose->coords pose))))))

(setq *range-min* nil)
(ros::subscribe "/basescan_range_min" std_msgs::Float32
                #'(lambda (msg) 
                    (setq *range-min* (send msg :data)))) 

#|
(ros::spin-once)
(print "hoge")
(unix:sleep 3)
|#

(defun compute-direction (co)
  (let* ((pos (send co :worldpos))
         (x (aref pos 0))
         (y (aref pos 1))
         (deg (+ -90 (rad2deg (atan2 x y)))))
    (- deg)))


(defun move ()
  (ros::advertise "/base_controller/command" geometry_msgs::Twist 1)
  (speak-jp "動きます")
  (loop 
    (setq *msg* (instance geometry_msgs::Twist :init))
    (send *msg* :linear :x 0.2)
    (send *msg* :angular :z 0)
    (ros::publish "/base_controller/command" *msg*)
    (unix:usleep 200000)
    (ros::spin-once)
    (send *fetch* :head_pan_joint 
          :joint-angle (compute-direction *co-fridge*))
    (transmit *fetch* :wait? nil :duration 1000)
    (when (< *range-min* 0.45)
      (return))))
(move)




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





