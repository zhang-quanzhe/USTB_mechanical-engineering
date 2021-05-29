;; Auto-generated. Do not edit!


(when (boundp 'car_navigation::PoseAngle)
  (if (not (find-package "CAR_NAVIGATION"))
    (make-package "CAR_NAVIGATION"))
  (shadow 'PoseAngle (find-package "CAR_NAVIGATION")))
(unless (find-package "CAR_NAVIGATION::POSEANGLE")
  (make-package "CAR_NAVIGATION::POSEANGLE"))

(in-package "ROS")
;;//! \htmlinclude PoseAngle.msg.html


(defclass car_navigation::PoseAngle
  :super ros::object
  :slots (_x _y _angle _z ))

(defmethod car_navigation::PoseAngle
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:angle __angle) 0.0)
    ((:z __z) 0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _angle (float __angle))
   (setq _z (round __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _angle
    4
    ;; uint8 _z
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _z
       (write-byte _z s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _z
     (setq _z (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get car_navigation::PoseAngle :md5sum-) "2bc475458b5903331d7038ac20f26588")
(setf (get car_navigation::PoseAngle :datatype-) "car_navigation/PoseAngle")
(setf (get car_navigation::PoseAngle :definition-)
      "float32 x
float32 y
float32 angle
uint8 z
")



(provide :car_navigation/PoseAngle "2bc475458b5903331d7038ac20f26588")


