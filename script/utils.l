(defun speak-wait (str)
  (speak-jp str)
  (send *ri* :wait-interpolation))

(defun reflect (robot)
  (send robot :angle-vector (send *ri* :state :potentio-vector)))

(defun transmit (robot &key (duration 10000) (force? t) (wait? nil))
  (if force?
      (send *ri* :angle-vector (send robot :angle-vector) duration)
      (send *ri* :angle-vector-motion-plan (send robot :angle-vector))
      )
  (if wait?
      (unix:usleep (* 1000 duration)))
  )

(defun set-home-posture (robot)
  (send *ri* :stop-grasp)
  (send robot :reset-pose)
  (send robot :torso_lift_joint :joint-angle 300) 
  (send robot :head_tilt_joint :joint-angle 0))

(defun show (robot)
  (let ((co-show (arrow)))
    (send co-show :newcoords (send *co* :copy-worldcoords))
    (objects (list co-show robot *fridge*))))

(defun create-fridge-colobj (co)
  (let* ((cube (make-cube 600 550 1600)))
    (send cube :newcoords (send co :copy-worldcoords))
    (send cube :translate #f(300 -180 -300) :local)
    cube))

(defun add-object-to-moveit (colobj obj)
  (send colobj :wipe-all)
  (send colobj :add-object obj :frame-id "base_footprint"
        :relative-pose (send obj :copy-worldcoords)
        :object-id "fridge"))


(defun pointmsg->vector (pointmsg)
  (float-vector 
    (send pointmsg :x)
    (send pointmsg :y)
    (send pointmsg :z)))

(defun compute-normal-vector (polygon) ;; polygon-msg
  (let* (
         (pts (send polygon :points))
         (n-pts (length pts))
         (p0 (pointmsg->vector (elt pts 0)))
         (p1 (pointmsg->vector (elt pts (/ n-pts 3))))
         (p2 (pointmsg->vector (elt pts (* 2 (/ n-pts 3)))))
         (vec0 (normalize-vector (v- p1 p0)))
         (vec1 (normalize-vector (v- p2 p1)))
         (vec-normal (v- (normalize-vector (v* vec0 vec1))))
         )
    vec-normal))
