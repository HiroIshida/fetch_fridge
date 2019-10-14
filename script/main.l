(load "package://fetcheus/fetch-interface.l")
(load "utils.l")
(require "models/arrow-object.l")
(ros::load-ros-manifest "fetch_fridge")
(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)
(ros::rate 10)

(fetch-init)
(objects *fetch*)
(print "hoge")

(setq *tfl* (instance ros::transform-listener :init))
(setq *colobj* (instance collision-object-publisher :init))
(send *colobj* :wipe-all)

(defun debug-print (str)
  (dotimes (i 30)
    (print str)))

;; DEFINE SUBSCRIBERS
(setq *co* nil)
(setq *fridge* nil)
(ros::subscribe "/handle_pose" fetch_fridge::PoseStampedBooled
                #'(lambda (msg) 
                    (let ((isValid (send msg :isvalid :data)))
                      (unless (null isValid)
                        (let* ((pose (send msg :ps :pose))
                               (lt (send *tfl* :lookup-transform
                                         "base_link" "/head_camera_rgb_optical_frame"
                                         (ros::time 0))))
                          (setq *co* (send lt :transform (ros::tf-pose->coords pose)))
                          (setq *fridge* (create-fridge-colobj *co*)))))))

#|
(setq *range-min* nil)
(ros::subscribe "/basescan_range_min" std_msgs::Float32
                #'(lambda (msg) 
                    (setq *range-min* (send msg :data))))
|#


(setq *pregrasp-pose* #f(300.432 -40.0 -11.8259 100.0 128.973 -173.109 54.3958 -106.06 -0.317235 0.56009))

(defun estimate-direction ()
  (ros::spin-once)
  (let* ((vec (transform (send *co* :worldrot) #f(1 0 0))))
    (setf (aref vec 2) 0.0)
    (setq vec (scale (/ -1.0 (norm vec)) vec))
    vec))

(defun compute-eye-deg-to-fridge (co)
  (let* ((vec (send co :worldpos))
         (x (aref vec 0))
         (y (aref vec 1))
         (deg (rad2deg (atan2 y x))))
    deg))

(defun feedback ()
  (let ((vec (estimate-direction)))
    (send *fetch* :rarm :move-end-pos (scale 50.0 vec))
    (transmit *fetch* :wait? nil :duration 2000)
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (unix:usleep 1000000)
    ))

(defun grasp-procedure ()
  (unix:sleep 2)
  (ros::spin-once)
  (add-object-to-moveit *colobj* *fridge*)
  (send *fetch* :angle-vector *pregrasp-pose*)
  (transmit *fetch* :wait? t :duration 5000)

  (print "a")
  (send *fetch* :rarm :inverse-kinematics
        (send *co* :copy-worldcoords)
        :rotation-axis t 
        :check-collision t
        :use-torso nil)
  (send *fetch* :rarm :move-end-pos #f(-150 0 0) :local)
  (print "b")
  (transmit *fetch* :wait? t :duration 2000)
  (print "c")

  ;; final reaching
  (send *fetch* :rarm :move-end-pos #f(140 0 0) :local)
  (send *colobj* :wipe-all)
  (transmit *fetch* :wait? t :duration 2000)
  (send *ri* :go-grasp :effort 300)
  )

(defun lookaround (robot)
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
        (return)))))

#|
(send *ri* :go-grasp :effort 100 :pos 0.1)
(send *ri* :wait-interpolation)
(set-home-posture *fetch*)
(transmit *fetch* :wait? t :duration 5000)
(setq *co* nil)
(lookaround *fetch*)
(objects (list *fetch* *fridge*))
(grasp-procedure)
(dotimes (i 10)
|#
(dotimes (i 20)
  (feedback))

