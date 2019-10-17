(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
(load "utils.l")
(require "models/arrow-object.l")
(ros::load-ros-manifest "fetch_fridge")
(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)
(pr2-init)

(setq *tfl* (instance ros::transform-listener :init))
(setq *colobj* (instance collision-object-publisher :init))
(send *colobj* :wipe-all)

;; DEFINE SUBSCRIBERS
(setq *co* nil)
(setq *fridge* nil)
(setq *lt* nil)

(ros::subscribe "/handle_pose" fetch_fridge::PoseStampedBooled
                #'(lambda (msg) 
                    (let ((isValid (send msg :isvalid :data)))
                      (unless (null isValid)
                        (let* ((pose (send msg :ps :pose))
                               (lt (send *tfl* :lookup-transform
                                         "base_link" "/head_mount_kinect_rgb_optical_frame"
                                         (ros::time 0))))
                          (setq *lt* lt)
                          (setq *co* (send lt :transform (ros::tf-pose->coords pose)))
                          (setq *fridge* (create-fridge-colobj *co*)))))))

(setq *preinsert-pose* #f(292.574 78.8998 0.089116 -93.3032 73.7924 5.19982 91.5822 87.7586 -47.7205 1.08744))

(setq *pregrasp-pose* #f(284.22 -84.2692 -4.37133 79.27 72.6499 16.9772 87.6052 -104.129 -5.39293 26.8394))


(defun lookaround (robot)
  (send *colobj* :wipe-all)
  (send robot :head_tilt_joint :joint-angle 0)
  (send robot :head_pan_joint :joint-angle 50)
  (transmit robot :wait? t :duration 1000)
  (unix:sleep 1) 
  (send robot :head_pan_joint :joint-angle -50)
  (let ((t-lookaround 10000))
    (transmit robot :wait? nil :duration t-lookaround)
    (setq *co* nil)
    (loop
      (print "====================================")
      (print *co*)
      (ros::spin-once)
      (unless (null *co*) 
        (send *ri* :stop-motion)
        (reflect robot)
        (add-object-to-moveit *colobj* *fridge*)
        (speak-wait "見つかりました")
        (return)))))
(lookaround *pr2*)


