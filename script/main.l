(load "package://fetcheus/fetch-interface.l")
(load "utils.l")
(require "models/arrow-object.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)

(fetch-init)
(objects *fetch*)
(print "hoge")

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

(setq *pregrasp-pose* #f(300.432 79.6908 4.72535 -105.784 118.111 -136.282 -20.4344 -138.45 -0.317235 0.56009))

(defun estimate-direction ()
  (ros::spin-once)
  (let* ((vec (transform (send *co* :worldrot) #f(1 0 0))))
    (setf (aref vec 2) 0.0)
    (setq vec (scale (/ -1.0 (norm vec)) vec))
    vec))

(defun feedback ()
  (let ((vec (estimate-direction)))
    (send *fetch* :rarm :move-end-pos (scale 20.0 vec))
    (transmit *fetch* :wait? t :duration 2000)))

(defun grasp-procedure ()
  (send *fetch* :angle-vector *pregrasp-pose*)
  (transmit *fetch* :wait? t)
  (ros::spin-once)
  (add-object-to-moveit *colobj* *fridge*)

  (send *ri* :go-grasp :effort 100 :pos 0.1)
  (send *fetch* :rarm :inverse-kinematics
        (send *co* :copy-worldcoords)
        :rotation-axis t 
        :use-torso nil)
  (send *fetch* :rarm :move-end-pos #f(-150 0 0) :local)
  (transmit *fetch*)
  (send *fetch* :rarm :move-end-pos #f(100 0 0) :local)
  (transmit *fetch* :wait? t)
  (send *colobj* :wipe-all)
  (send *fetch* :rarm :move-end-pos #f(50 0 0) :local)
  (transmit *fetch* :wait? t)
  (send *ri* :start-grasp :effort 300)
  )
(grasp-procedure)
(dotimes (i 20)
  (feedback))
















