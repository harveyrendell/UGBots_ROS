; Auto-generated. Do not edit!


(cl:in-package ugbots_ros-msg)


;//! \htmlinclude bin_status.msg.html

(cl:defclass <bin_status> (roslisp-msg-protocol:ros-message)
  ((bin_x
    :reader bin_x
    :initarg :bin_x
    :type cl:float
    :initform 0.0)
   (bin_y
    :reader bin_y
    :initarg :bin_y
    :type cl:float
    :initform 0.0)
   (bin_stat
    :reader bin_stat
    :initarg :bin_stat
    :type cl:string
    :initform ""))
)

(cl:defclass bin_status (<bin_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bin_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bin_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ugbots_ros-msg:<bin_status> is deprecated: use ugbots_ros-msg:bin_status instead.")))

(cl:ensure-generic-function 'bin_x-val :lambda-list '(m))
(cl:defmethod bin_x-val ((m <bin_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ugbots_ros-msg:bin_x-val is deprecated.  Use ugbots_ros-msg:bin_x instead.")
  (bin_x m))

(cl:ensure-generic-function 'bin_y-val :lambda-list '(m))
(cl:defmethod bin_y-val ((m <bin_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ugbots_ros-msg:bin_y-val is deprecated.  Use ugbots_ros-msg:bin_y instead.")
  (bin_y m))

(cl:ensure-generic-function 'bin_stat-val :lambda-list '(m))
(cl:defmethod bin_stat-val ((m <bin_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ugbots_ros-msg:bin_stat-val is deprecated.  Use ugbots_ros-msg:bin_stat instead.")
  (bin_stat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bin_status>) ostream)
  "Serializes a message object of type '<bin_status>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bin_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bin_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'bin_stat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'bin_stat))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bin_status>) istream)
  "Deserializes a message object of type '<bin_status>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bin_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bin_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bin_stat) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'bin_stat) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bin_status>)))
  "Returns string type for a message object of type '<bin_status>"
  "ugbots_ros/bin_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bin_status)))
  "Returns string type for a message object of type 'bin_status"
  "ugbots_ros/bin_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bin_status>)))
  "Returns md5sum for a message object of type '<bin_status>"
  "9e7ec5f3609eb98fb2e1f6b8f4b7b677")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bin_status)))
  "Returns md5sum for a message object of type 'bin_status"
  "9e7ec5f3609eb98fb2e1f6b8f4b7b677")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bin_status>)))
  "Returns full string definition for message of type '<bin_status>"
  (cl:format cl:nil "float64 bin_x~%float64 bin_y~%string bin_stat~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bin_status)))
  "Returns full string definition for message of type 'bin_status"
  (cl:format cl:nil "float64 bin_x~%float64 bin_y~%string bin_stat~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bin_status>))
  (cl:+ 0
     8
     8
     4 (cl:length (cl:slot-value msg 'bin_stat))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bin_status>))
  "Converts a ROS message object to a list"
  (cl:list 'bin_status
    (cl:cons ':bin_x (bin_x msg))
    (cl:cons ':bin_y (bin_y msg))
    (cl:cons ':bin_stat (bin_stat msg))
))
