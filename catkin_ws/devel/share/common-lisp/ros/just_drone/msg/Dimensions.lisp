; Auto-generated. Do not edit!


(cl:in-package just_drone-msg)


;//! \htmlinclude Dimensions.msg.html

(cl:defclass <Dimensions> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Dimensions (<Dimensions>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dimensions>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dimensions)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name just_drone-msg:<Dimensions> is deprecated: use just_drone-msg:Dimensions instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Dimensions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader just_drone-msg:width-val is deprecated.  Use just_drone-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <Dimensions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader just_drone-msg:height-val is deprecated.  Use just_drone-msg:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dimensions>) ostream)
  "Serializes a message object of type '<Dimensions>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dimensions>) istream)
  "Deserializes a message object of type '<Dimensions>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dimensions>)))
  "Returns string type for a message object of type '<Dimensions>"
  "just_drone/Dimensions")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dimensions)))
  "Returns string type for a message object of type 'Dimensions"
  "just_drone/Dimensions")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dimensions>)))
  "Returns md5sum for a message object of type '<Dimensions>"
  "a4069c3fe743f7881e847e9b0ab5556a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dimensions)))
  "Returns md5sum for a message object of type 'Dimensions"
  "a4069c3fe743f7881e847e9b0ab5556a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dimensions>)))
  "Returns full string definition for message of type '<Dimensions>"
  (cl:format cl:nil "int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dimensions)))
  "Returns full string definition for message of type 'Dimensions"
  (cl:format cl:nil "int32 width~%int32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dimensions>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dimensions>))
  "Converts a ROS message object to a list"
  (cl:list 'Dimensions
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
