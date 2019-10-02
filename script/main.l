(load "package://fetcheus/fetch-interface.l")
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

(send *fetch* :torso_lift_joint :joint-angle 300) 
(send *fetch* :head_tilt_joint :joint-angle 0) 
(send *fetch* :rarm :inverse-kinematics
      (make-coords :pos #f(710 0 1150) :rpy #f(0 0 0))
      :rotation-axis t 
      :use-torso nil)

(send robot arm? :inverse-kinematics 
                 (make-coords :pos pos :rpy (float-vector (deg2rad (+ 90 deg)) 0 (/ pi 2)))
                 :rotation-axis t :use-torso nil)

