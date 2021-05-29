; Auto-generated. Do not edit!


(cl:in-package car_navigation-msg)


;//! \htmlinclude PoseAngle.msg.html

(cl:defclass <PoseAngle> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PoseAngle (<PoseAngle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseAngle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseAngle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name car_navigation-msg:<PoseAngle> is deprecated: use car_navigation-msg:PoseAngle instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <PoseAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car_navigation-msg:x-val is deprecated.  Use car_navigation-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <PoseAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car_navigation-msg:y-val is deprecated.  Use car_navigation-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <PoseAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car_navigation-msg:angle-val is deprecated.  Use car_navigation-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <PoseAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader car_navigation-msg:z-val is deprecated.  Use car_navigation-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseAngle>) ostream)
  "Serializes a message object of type '<PoseAngle>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'z)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseAngle>) istream)
  "Deserializes a message object of type '<PoseAngle>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'z)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseAngle>)))
  "Returns string type for a message object of type '<PoseAngle>"
  "car_navigation/PoseAngle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseAngle)))
  "Returns string type for a message object of type 'PoseAngle"
  "car_navigation/PoseAngle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseAngle>)))
  "Returns md5sum for a message object of type '<PoseAngle>"
  "2bc475458b5903331d7038ac20f26588")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseAngle)))
  "Returns md5sum for a message object of type 'PoseAngle"
  "2bc475458b5903331d7038ac20f26588")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseAngle>)))
  "Returns full string definition for message of type '<PoseAngle>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 angle~%uint8 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseAngle)))
  "Returns full string definition for message of type 'PoseAngle"
  (cl:format cl:nil "float32 x~%float32 y~%float32 angle~%uint8 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseAngle>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseAngle>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseAngle
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':z (z msg))
))
