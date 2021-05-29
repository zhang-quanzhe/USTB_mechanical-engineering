;; Auto-generated. Do not edit!


(when (boundp 'cancontrol::vandw)
  (if (not (find-package "CANCONTROL"))
    (make-package "CANCONTROL"))
  (shadow 'vandw (find-package "CANCONTROL")))
(unless (find-package "CANCONTROL::VANDW")
  (make-package "CANCONTROL::VANDW"))

(in-package "ROS")
;;//! \htmlinclude vandw.msg.html


(defclass cancontrol::vandw
  :super ros::object
  :slots (_v _w _dis_x _dis_y ))

(defmethod cancontrol::vandw
  (:init
   (&key
    ((:v __v) 0)
    ((:w __w) 0)
    ((:dis_x __dis_x) 0)
    ((:dis_y __dis_y) 0)
    )
   (send-super :init)
   (setq _v (round __v))
   (setq _w (round __w))
   (setq _dis_x (round __dis_x))
   (setq _dis_y (round __dis_y))
   self)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:dis_x
   (&optional __dis_x)
   (if __dis_x (setq _dis_x __dis_x)) _dis_x)
  (:dis_y
   (&optional __dis_y)
   (if __dis_y (setq _dis_y __dis_y)) _dis_y)
  (:serialization-length
   ()
   (+
    ;; int32 _v
    4
    ;; int32 _w
    4
    ;; int32 _dis_x
    4
    ;; int32 _dis_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _v
       (write-long _v s)
     ;; int32 _w
       (write-long _w s)
     ;; int32 _dis_x
       (write-long _dis_x s)
     ;; int32 _dis_y
       (write-long _dis_y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _v
     (setq _v (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _w
     (setq _w (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _dis_x
     (setq _dis_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _dis_y
     (setq _dis_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cancontrol::vandw :md5sum-) "d7fadd1e689290e03c5298e42f0e94d6")
(setf (get cancontrol::vandw :datatype-) "cancontrol/vandw")
(setf (get cancontrol::vandw :definition-)
      "int32 v
int32 w
int32 dis_x
int32 dis_y
")



(provide :cancontrol/vandw "d7fadd1e689290e03c5298e42f0e94d6")


