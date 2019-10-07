(load "utils.l")
(setq *use-fetch* t)
(if *use-fetch*
    (progn
      (load "package://fetcheus/fetch-interface.l")
      (fetch-init)
      )
    (progn
      (load "package://pr2eus/pr2-interface.l")
      (pr2-init)))
(setq *camera-frame* (if *use-fetch* 
                         "/head_camera_rgb_optical_frame"
                         "/head_mount_kinect_rgb_optical_frame"))
