; Auto-generated. Do not edit!


(cl:in-package ultra_serial_port-msg)


;//! \htmlinclude Ultrasound.msg.html

(cl:defclass <Ultrasound> (roslisp-msg-protocol:ros-message)
  ((dis
    :reader dis
    :initarg :dis
    :type cl:fixnum
    :initform 0)
   (dis2
    :reader dis2
    :initarg :dis2
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Ultrasound (<Ultrasound>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ultrasound>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ultrasound)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ultra_serial_port-msg:<Ultrasound> is deprecated: use ultra_serial_port-msg:Ultrasound instead.")))

(cl:ensure-generic-function 'dis-val :lambda-list '(m))
(cl:defmethod dis-val ((m <Ultrasound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ultra_serial_port-msg:dis-val is deprecated.  Use ultra_serial_port-msg:dis instead.")
  (dis m))

(cl:ensure-generic-function 'dis2-val :lambda-list '(m))
(cl:defmethod dis2-val ((m <Ultrasound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ultra_serial_port-msg:dis2-val is deprecated.  Use ultra_serial_port-msg:dis2 instead.")
  (dis2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ultrasound>) ostream)
  "Serializes a message object of type '<Ultrasound>"
  (cl:let* ((signed (cl:slot-value msg 'dis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dis2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ultrasound>) istream)
  "Deserializes a message object of type '<Ultrasound>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dis) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dis2) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ultrasound>)))
  "Returns string type for a message object of type '<Ultrasound>"
  "ultra_serial_port/Ultrasound")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ultrasound)))
  "Returns string type for a message object of type 'Ultrasound"
  "ultra_serial_port/Ultrasound")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ultrasound>)))
  "Returns md5sum for a message object of type '<Ultrasound>"
  "fbe7339e94fec895366dda899719a997")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ultrasound)))
  "Returns md5sum for a message object of type 'Ultrasound"
  "fbe7339e94fec895366dda899719a997")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ultrasound>)))
  "Returns full string definition for message of type '<Ultrasound>"
  (cl:format cl:nil "int16 dis~%int16 dis2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ultrasound)))
  "Returns full string definition for message of type 'Ultrasound"
  (cl:format cl:nil "int16 dis~%int16 dis2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ultrasound>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ultrasound>))
  "Converts a ROS message object to a list"
  (cl:list 'Ultrasound
    (cl:cons ':dis (dis msg))
    (cl:cons ':dis2 (dis2 msg))
))
