(load "package://fetcheus/fetch-interface.l")
(load "utils.l")
(require "models/arrow-object.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)

(fetch-init)
(objects *fetch*)
(setq *tfl* (instance ros::transform-listener :init))
(setq *colobj* (instance collision-object-publisher :init))
(send *colobj* :wipe-all)

(setq *co* nil)
(setq *fridge* nil)
(ros::subscribe "/head_camera_remote/rgb/object_pose" geometry_msgs::PoseStamped
                #'(lambda (msg) 
                    (let* ((pose (send msg :pose))
                           (lt (send *tfl* :lookup-transform
                                     "base_link" "/head_camera_rgb_optical_frame"
                                     (ros::time 0))))
                      (setq *co* (send lt :transform (ros::tf-pose->coords pose)))
                      (setq *fridge* (create-fridge-colobj *co*)))))

#|
(defun add-fridge-moveit (co)
  (let ((fridge (create-fridge-colobj co)))
    (send *colobj* :add-object fridge :frame-id "base_footprint" 
          :relative-pose (send fridge :copy-worldcoords)
          :object-id "fridge")))
|#

(set-home-posture *fetch*)
(transmit *fetch*)
(unix:sleep 2)
(ros::spin-once) ;; update *co*
(add-object-to-moveit *colobj* *fridge*)


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

;(setq *coords* (posemsg2coords *pose*))














#|
(send *fetch* 
(send *fetch* :torso_lift_joint :joint-angle 300) 
(send *fetch* :head_tilt_joint :joint-angle 0) 
(send *fetch* :rarm :inverse-kinematics
      (make-coords :pos #f(710 0 1150) :rpy #f(0 0 0))
      :rotation-axis t 
      :use-torso nil)
|#

