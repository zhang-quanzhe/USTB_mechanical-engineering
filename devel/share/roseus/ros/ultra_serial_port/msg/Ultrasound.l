;; Auto-generated. Do not edit!


(when (boundp 'ultra_serial_port::Ultrasound)
  (if (not (find-package "ULTRA_SERIAL_PORT"))
    (make-package "ULTRA_SERIAL_PORT"))
  (shadow 'Ultrasound (find-package "ULTRA_SERIAL_PORT")))
(unless (find-package "ULTRA_SERIAL_PORT::ULTRASOUND")
  (make-package "ULTRA_SERIAL_PORT::ULTRASOUND"))

(in-package "ROS")
;;//! \htmlinclude Ultrasound.msg.html


(defclass ultra_serial_port::Ultrasound
  :super ros::object
  :slots (_dis _dis2 ))

(defmethod ultra_serial_port::Ultrasound
  (:init
   (&key
    ((:dis __dis) 0)
    ((:dis2 __dis2) 0)
    )
   (send-super :init)
   (setq _dis (round __dis))
   (setq _dis2 (round __dis2))
   self)
  (:dis
   (&optional __dis)
   (if __dis (setq _dis __dis)) _dis)
  (:dis2
   (&optional __dis2)
   (if __dis2 (setq _dis2 __dis2)) _dis2)
  (:serialization-length
   ()
   (+
    ;; int16 _dis
    2
    ;; int16 _dis2
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _dis
       (write-word _dis s)
     ;; int16 _dis2
       (write-word _dis2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _dis
     (setq _dis (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _dis2
     (setq _dis2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get ultra_serial_port::Ultrasound :md5sum-) "fbe7339e94fec895366dda899719a997")
(setf (get ultra_serial_port::Ultrasound :datatype-) "ultra_serial_port/Ultrasound")
(setf (get ultra_serial_port::Ultrasound :definition-)
      "int16 dis
int16 dis2

")



(provide :ultra_serial_port/Ultrasound "fbe7339e94fec895366dda899719a997")


