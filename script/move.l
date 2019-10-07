(load "load-common.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "move" :anonymous t)


(ros::advertise "/base_controller/command" geometry_msgs::Twist 1)
(defun move ()
  (loop 
    (setq *msg* (instance geometry_msgs::Twist :init))
    (send *msg* :linear :x 0.2)
    (send *msg* :angular :z 0)
    (ros::publish "/base_controller/command" *msg*)
    (when *use-fetch*
      (unix:usleep 200000))
    (ros::spin-once)))
(move)


