(load "load-common.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "move" :anonymous t)

(setq *range-min* nil)
(ros::subscribe "/basescan_range_min" std_msgs::Float32
                #'(lambda (msg) 
                    (setq *range-min* (send msg :data)))) 

(ros::advertise "/base_controller/command" geometry_msgs::Twist 1)
(defun move ()
  (loop 
    (print *range-min*)
    (setq *msg* (instance geometry_msgs::Twist :init))
    (send *msg* :linear :x 0.1)
    (send *msg* :angular :z 0)
    (ros::publish "/base_controller/command" *msg*)
    (when *use-fetch*
      (unix:usleep 200000))
    (ros::spin-once)
    (when (and (< 0.1 *range-min*) (< *range-min* 0.4))
      (return))
    ))
(move)


