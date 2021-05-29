; Auto-generated. Do not edit!


(cl:in-package cancontrol-msg)


;//! \htmlinclude vandw.msg.html

(cl:defclass <vandw> (roslisp-msg-protocol:ros-message)
  ((v
    :reader v
    :initarg :v
    :type cl:integer
    :initform 0)
   (w
    :reader w
    :initarg :w
    :type cl:integer
    :initform 0)
   (dis_x
    :reader dis_x
    :initarg :dis_x
    :type cl:integer
    :initform 0)
   (dis_y
    :reader dis_y
    :initarg :dis_y
    :type cl:integer
    :initform 0))
)

(cl:defclass vandw (<vandw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vandw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vandw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cancontrol-msg:<vandw> is deprecated: use cancontrol-msg:vandw instead.")))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <vandw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cancontrol-msg:v-val is deprecated.  Use cancontrol-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <vandw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cancontrol-msg:w-val is deprecated.  Use cancontrol-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'dis_x-val :lambda-list '(m))
(cl:defmethod dis_x-val ((m <vandw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cancontrol-msg:dis_x-val is deprecated.  Use cancontrol-msg:dis_x instead.")
  (dis_x m))

(cl:ensure-generic-function 'dis_y-val :lambda-list '(m))
(cl:defmethod dis_y-val ((m <vandw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cancontrol-msg:dis_y-val is deprecated.  Use cancontrol-msg:dis_y instead.")
  (dis_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vandw>) ostream)
  "Serializes a message object of type '<vandw>"
  (cl:let* ((signed (cl:slot-value msg 'v)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'w)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dis_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dis_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vandw>) istream)
  "Deserializes a message object of type '<vandw>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'v) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'w) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dis_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dis_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vandw>)))
  "Returns string type for a message object of type '<vandw>"
  "cancontrol/vandw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vandw)))
  "Returns string type for a message object of type 'vandw"
  "cancontrol/vandw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vandw>)))
  "Returns md5sum for a message object of type '<vandw>"
  "d7fadd1e689290e03c5298e42f0e94d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vandw)))
  "Returns md5sum for a message object of type 'vandw"
  "d7fadd1e689290e03c5298e42f0e94d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vandw>)))
  "Returns full string definition for message of type '<vandw>"
  (cl:format cl:nil "int32 v~%int32 w~%int32 dis_x~%int32 dis_y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vandw)))
  "Returns full string definition for message of type 'vandw"
  (cl:format cl:nil "int32 v~%int32 w~%int32 dis_x~%int32 dis_y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vandw>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vandw>))
  "Converts a ROS message object to a list"
  (cl:list 'vandw
    (cl:cons ':v (v msg))
    (cl:cons ':w (w msg))
    (cl:cons ':dis_x (dis_x msg))
    (cl:cons ':dis_y (dis_y msg))
))
