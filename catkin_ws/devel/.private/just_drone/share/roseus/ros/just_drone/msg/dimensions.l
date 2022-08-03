;; Auto-generated. Do not edit!


(when (boundp 'just_drone::dimensions)
  (if (not (find-package "JUST_DRONE"))
    (make-package "JUST_DRONE"))
  (shadow 'dimensions (find-package "JUST_DRONE")))
(unless (find-package "JUST_DRONE::DIMENSIONS")
  (make-package "JUST_DRONE::DIMENSIONS"))

(in-package "ROS")
;;//! \htmlinclude dimensions.msg.html


(defclass just_drone::dimensions
  :super ros::object
  :slots (_width _height ))

(defmethod just_drone::dimensions
  (:init
   (&key
    ((:width __width) 0)
    ((:height __height) 0)
    )
   (send-super :init)
   (setq _width (round __width))
   (setq _height (round __height))
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; int32 _width
    4
    ;; int32 _height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _width
       (write-long _width s)
     ;; int32 _height
       (write-long _height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get just_drone::dimensions :md5sum-) "a4069c3fe743f7881e847e9b0ab5556a")
(setf (get just_drone::dimensions :datatype-) "just_drone/dimensions")
(setf (get just_drone::dimensions :definition-)
      "int32 width
int32 height

")



(provide :just_drone/dimensions "a4069c3fe743f7881e847e9b0ab5556a")

