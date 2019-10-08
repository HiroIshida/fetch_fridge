(load "package://fetcheus/fetch-interface.l")
(load "utils.l")
(require "models/arrow-object.l")
(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)
(ros::rate 10)

(fetch-init)
(objects *fetch*)
(print "hoge")

(setq *tfl* (instance ros::transform-listener :init))
(setq *colobj* (instance collision-object-publisher :init))
(send *colobj* :wipe-all)

;; DEFINE SUBSCRIBERS
(setq *co* nil)
(setq *fridge* nil)
(ros::subscribe "/handle_pose" geometry_msgs::PoseStamped
                #'(lambda (msg) 
                    (let* ((pose (send msg :pose))
                           (lt (send *tfl* :lookup-transform
                                     "base_link" "/head_camera_rgb_optical_frame"
                                     (ros::time 0))))
                      (setq *co* (send lt :transform (ros::tf-pose->coords pose)))
                      (setq *fridge* (create-fridge-colobj *co*)))))

(setq *range-min* nil)
(ros::subscribe "/basescan_range_min" std_msgs::Float32
                #'(lambda (msg) 
                    (setq *range-min* (send msg :data))))


(setq *pregrasp-pose* #f(300.432 -40.0 -11.8259 100.0 128.973 -173.109 54.3958 -106.06 -0.317235 0.56009))
(send *ri* :angle-vector *pregrasp-pose*)

(defun estimate-direction ()
  (ros::spin-once)
  (let* ((vec (transform (send *co* :worldrot) #f(1 0 0))))
    (setf (aref vec 2) 0.0)
    (setq vec (scale (/ -1.0 (norm vec)) vec))
    vec))

(defun feedback ()
  (let ((vec (estimate-direction)))
    (send *fetch* :rarm :move-end-pos (scale 50.0 vec))
    (transmit *fetch* :wait? nil :duration 2000)
    (unix:usleep 1000000)
    ))

(defun grasp-procedure ()
  (send *fetch* :angle-vector *pregrasp-pose*)
  (transmit *fetch* :wait? t :duration 5000)
  (ros::spin-once)
  (add-object-to-moveit *colobj* *fridge*)

  (send *fetch* :rarm :inverse-kinematics
        (send *co* :copy-worldcoords)
        :rotation-axis t 
        :use-torso nil)
  (send *fetch* :rarm :move-end-pos #f(-150 0 0) :local)
  (transmit *fetch* :wait? t :duration 2000)

  ;; final reaching
  (send *fetch* :rarm :move-end-pos #f(140 0 0) :local)
  (send *colobj* :wipe-all)
  (transmit *fetch* :wait? t :duration 2000)
  (send *ri* :go-grasp :effort 300)
  )

;(ros::spin-once)
(send *ri* :go-grasp :effort 100 :pos 0.1)
(send *ri* :wait-interpolation)
(set-home-posture *fetch*)
(transmit *fetch* :wait? t :duration 5000)
(loop 
  (print "hoge")
  (ros::spin-once)
  (unix:usleep 500000)
  (unless (null *co*) (return))
  )
(speak-jp "あ")
(objects (list *fetch* *fridge*))
(grasp-procedure)
(dotimes (i 100)
  (feedback)
  )




