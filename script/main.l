(load "package://fetcheus/fetch-interface.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)

(fetch-init)
(objects *fetch*)

(defun reflect (robot)
  (send robot :angle-vector (send *ri* :state :potentio-vector)))

(defun transmit (robot &key (duration 10000) (force? t) (wait? nil))
  (if force?
      (send *ri* :angle-vector (send robot :angle-vector) duration)
      (send *ri* :angle-vector-motion-plan (send robot :angle-vector))
      )
  (if wait?
    (send *ri* :wait-interpolation))
  )

(setq *pos-fridge* nil)
(ros::subscribe "/head_camera_remote/pose_fridge_handle" geometry_msgs::PoseStamped
                #'(lambda (msg) 
                    (let* ((pose (send msg :pose))
                           (pos (send pose :position))
                           (vec (float-vector
                                  (* (send pos :x) 1000)
                                  (* (send pos :y) 1000)
                                  (* (send pos :z) 1000))))
                      (setq *pos-fridge* vec))))

(defun visualize ()
  (let ((cube (make-cube 10 10 10)))
    (send cube :translate *pos-fridge* :world)
    (objects (list *fetch* cube))))

(defun set-home-posture (robot)
  (send *ri* :stop-grasp)
  (send robot :reset-pose)
  (send robot :torso_lift_joint :joint-angle 300) 
  (send robot :head_tilt_joint :joint-angle 0))

(defun main ()
  (set-home-posture *fetch*)
  (transmit *fetch*)
  (unix::sleep 2)
  (ros::spin-once)

  (send *ri* :go-grasp :effort 100 :pos 0.1)
  (send *fetch* :rarm :inverse-kinematics
        (make-coords :pos *pos-fridge* :rpy #f(0 0 0))
        :rotation-axis t 
        :use-torso nil)
  (send *fetch* :rarm :move-end-pos #f(-200 0 0) :local)
  (transmit *fetch*)
  (send *ri* :go-grasp :effort 100 :pos 0.05)

  (send *fetch* :rarm :move-end-pos #f(180 0 0) :local)
  (transmit *fetch*)
  (send *ri* :start-grasp :effort 300)

  (send *fetch* :rarm :move-end-pos #f(-180 0 0) :local)
  (transmit *fetch*)

  (send *fetch* :rarm :move-end-pos #f(-100 0 0) :local)
  (transmit *fetch*)
  )







#|
(send *fetch* 
(send *fetch* :torso_lift_joint :joint-angle 300) 
(send *fetch* :head_tilt_joint :joint-angle 0) 
(send *fetch* :rarm :inverse-kinematics
      (make-coords :pos #f(710 0 1150) :rpy #f(0 0 0))
      :rotation-axis t 
      :use-torso nil)
|#

