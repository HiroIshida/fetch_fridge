(load "package://pr2eus/pr2-interface.l")
;(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")

(load "utils.l")
(require "models/arrow-object.l")
(ros::load-ros-manifest "fetch_fridge")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "roseus")
(ros::roseus "demo_main" :anonymous t)
(pr2-init)

(setq *tfl* (instance ros::transform-listener :init))
;(setq *colobj* (instance collision-object-publisher :init))
;(send *colobj* :wipe-all)


;; DEFINE SUBSCRIBERS
(setq *msg* nil)
(setq *vec-normal* nil)
(ros::subscribe "/fridge_cloud/multi_plane_estimate/output_polygon" jsk_recognition_msgs::PolygonArray
                #'(lambda (msg) 
                    (setq *msg* msg)
                    (print "hoge")
                    (unless (null (send msg :polygons))
                      (let* (
                             (poly-door (send (elt (send msg :polygons) 0) :polygon))
                             (vec-normal (compute-normal-vector poly-door))
                             )
                        (setq *vec-normal* vec-normal)
                        (print *vec-normal*)
                        ))))

(setq *co* nil)
(setq *fridge* nil)
(setq *lt* nil)
(ros::subscribe "/handle_pose" fetch_fridge::PoseStampedBooled
                #'(lambda (msg) 
                    (let ((isValid (send msg :isvalid :data)))
                      (let* ((pose (send msg :ps :pose))
                             (lt (send *tfl* :lookup-transform
                                       "base_link" "/head_mount_kinect_rgb_optical_frame"
                                       (ros::time 0))))
                        (setq *lt* lt)
                        (setq *co* (send lt :transform (ros::tf-pose->coords pose)))
                        (setq *fridge* (create-fridge-colobj *co*))))))

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

(defun feedback (robot)
  (ros::spin-once)
  (let* ((x (aref *vec-normal* 0))
         (y (aref *vec-normal* 1))
         (rad (atan2 y x)))
    (let* ((co_end (send robot :rarm :end-coords))
           (co (make-coords)))
      (print *vec-normal*)
      (send co :translate (send co_end :worldpos))
      (send co :translate (scale -5.0 *vec-normal*))
      (send co :rotate rad :z :local)
      (send robot :rarm :inverse-kinematics
            co
            :rotation-axis t
            :check-collision nil
            :use-torso nil)
      (transmit robot :wait? nil :duration 1000)
      (unix:usleep 300000)
      )))


(setq *preinsert-pose* #f(292.574 78.8998 0.089116 -93.3032 73.7924 5.19982 91.5822 87.7586 -47.7205 1.08744))
(setq *pregrasp-pose* #f(284.22 -84.2692 -4.37133 79.27 72.6499 16.9772 87.6052 -104.129 -5.39293 26.8394))


(defun lookaround (robot)
  ;(send *colobj* :wipe-all)
  (send robot :head_tilt_joint :joint-angle 0)
  (send robot :head_pan_joint :joint-angle 30)
  (transmit robot :wait? t :duration 1000)
  (unix:sleep 1) 
  (send robot :head_pan_joint :joint-angle -30)
  (let ((t-lookaround 20000))
    (transmit robot :wait? nil :duration t-lookaround)
    (setq *co* nil)
    (loop
      (print "====================================")
      (print *co*)
      (ros::spin-once)
      (unless (null *co*) 
        (send *ri* :stop-motion)
        (reflect robot)
        ;(add-object-to-moveit *colobj* *fridge*)
        (speak-wait "見つかりました")
        (return)))))

(defun lookfridge (robot)
  (let ((deg (compute-eye-deg-to-fridge *co*)))
    (send robot :head_pan_joint :joint-angle (- deg 20))
    (transmit robot :wait? t :duration 1000)
    (unix:sleep 3)
    (ros::spin-once)
    ;(add-object-to-moveit *colobj* *fridge*)
    ))

(setq *deb* nil)
(defun insert-procedure (robot)
  (ros::spin-once)
  ;(add-object-to-moveit *colobj* *fridge*)
  (let ((co (send *co* :copy-worldcoords)))
    (send co :rotate 0.2 :z) 
    (send co :translate #f(-100 30 0) :local)
    (send robot :rarm :inverse-kinematics
          co
          :rotation-axis t
          :check-collision nil
          :use-torso nil)
    (setq *deb* co)
    ;(send robot :rarm :move-end-pos #f(-100 -100 0) :local)
    ;(transmit robot :wait? t :duration 10000)
    )
  )

(unix:sleep 1)
(ros::spin-once)

(setq *base-pose* #f(11.5 38.291 -0.248724 102.662 -121.525 155.162 -19.7487 411.196 -47.4454 -20.1768 -49.2259 -121.4 226.338 -72.238 -467.586 0 0))

(print "hoge")
(send *pr2* :angle-vector *base-pose*)
(transmit *pr2* :duration 2000 :wait? t)

#|
(lookaround *pr2*)
(speak-wait "みます")
(dotimes (i 3)
  (lookfridge *pr2*))
|#

(send *ri* :stop-grasp)
(ros::spin-once)
(insert-procedure *pr2*)
(transmit *pr2* :duration 2000 :wait? t)
(print "hoge")

(send *pr2* :rarm :move-end-pos #f(100 0 0) :local)
(transmit *pr2* :duration 2000 :wait? t)
(send *ri* :start-grasp :rarm :gain 0.05)
(send *ri* :wait-interpolation)
(speak-jp "はじめます")
(dotimes (i 100)
  (feedback *pr2*))
(send *ri* :stop-grasp)
