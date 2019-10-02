(load "package://fetcheus/fetch-interface.l")
(require "models/arrow-object.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)

(fetch-init)
(objects *fetch*)
(setq *tfl* (instance ros::transform-listener :init))

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
(setq *q-fridge* nil)
(setq *co* nil)
(Setq *pose* nil)
(ros::subscribe "/head_camera_remote/rgb/object_pose" geometry_msgs::PoseStamped
                #'(lambda (msg) 
                    (let* ((pose_ (send msg :pose)))
                      (setq *pose* pose_))))


(setq *lt* (send *tfl* :lookup-transform 
                 "base_link" "/head_camera_rgb_optical_frame"
                 (ros::time 0)))
(setq *co-new* (send *lt* :transform (ros::tf-pose->coords *pose*)))


#|
(lt (send *tfl* :lookup-transform 
         "/base_link" 
         "/head_camera_rgb_optical_frame" (ros::time 0)))
|#

(defun visualizable (_co)
  (let* ((co (arrow))
         (send co :newcoords (send _co :copy-worldcoords)))
    co))


#|
(defun visualize ()
  (let ((cube (make-cube 10 10 10)))
    (send cube :translate *pos-fridge* :world)
    (objects (list *fetch* cube))))
|#

(defun set-home-posture (robot)
  (send *ri* :stop-grasp)
  (send robot :reset-pose)
  (send robot :torso_lift_joint :joint-angle 300) 
  (send robot :head_tilt_joint :joint-angle 0))

(defun posemsg2coords (pose-msg)
  (let* (
         (p__ (send pose-msg :position))
         (q_ (send pose-msg :orientation))
         (p_ (float-vector (send p__ :x) (send p__ :y) (send p__ :z)))
         (p (scale 1000 p_))
         (q (float-vector (send q_ :x) (send q_ :y) (send q_ :z) (send q_ :w)))
         (mat (transpose (quaternion2matrix q)))
         (rpy (first (rpy-angle mat)))
         )
    (make-coords :pos p :rpy rpy)))

;(ros::spin-once)

(defun tmp ()
  (setq *coo* (arrow))
  (send *coo* :newcoords (send *co-new* :copy-worldcoords))
  (objects (list *coo* *fetch*)))

        


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

