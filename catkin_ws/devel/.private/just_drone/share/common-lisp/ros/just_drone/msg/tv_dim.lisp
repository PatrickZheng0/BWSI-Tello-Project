; Auto-generated. Do not edit!


(cl:in-package just_drone-msg)


;//! \htmlinclude tv_dim.msg.html

(cl:defclass <tv_dim> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0))
)

(cl:defclass tv_dim (<tv_dim>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tv_dim>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tv_dim)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name just_drone-msg:<tv_dim> is deprecated: use just_drone-msg:tv_dim instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <tv_dim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader just_drone-msg:width-val is deprecated.  Use just_drone-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <tv_dim>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader just_drone-msg:height-val is deprecated.  Use just_drone-msg:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tv_dim>) ostream)
  "Serializes a message object of type '<tv_dim>"
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tv_dim>) istream)
  "Deserializes a message object of type '<tv_dim>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tv_dim>)))
  "Returns string type for a message object of type '<tv_dim>"
  "just_drone/tv_dim")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tv_dim)))
  "Returns string type for a message object of type 'tv_dim"
  "just_drone/tv_dim")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tv_dim>)))
  "Returns md5sum for a message object of type '<tv_dim>"
  "a4069c3fe743f7881e847e9b0ab5556a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tv_dim)))
  "Returns md5sum for a message object of type 'tv_dim"
  "a4069c3fe743f7881e847e9b0ab5556a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tv_dim>)))
  "Returns full string definition for message of type '<tv_dim>"
  (cl:format cl:nil "int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tv_dim)))
  "Returns full string definition for message of type 'tv_dim"
  (cl:format cl:nil "int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tv_dim>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tv_dim>))
  "Converts a ROS message object to a list"
  (cl:list 'tv_dim
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
