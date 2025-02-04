;; Auto-generated. Do not edit!


(when (boundp 'autonomous_navigation::Vector3Array)
  (if (not (find-package "AUTONOMOUS_NAVIGATION"))
    (make-package "AUTONOMOUS_NAVIGATION"))
  (shadow 'Vector3Array (find-package "AUTONOMOUS_NAVIGATION")))
(unless (find-package "AUTONOMOUS_NAVIGATION::VECTOR3ARRAY")
  (make-package "AUTONOMOUS_NAVIGATION::VECTOR3ARRAY"))

(in-package "ROS")
;;//! \htmlinclude Vector3Array.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass autonomous_navigation::Vector3Array
  :super ros::object
  :slots (_vectors ))

(defmethod autonomous_navigation::Vector3Array
  (:init
   (&key
    ((:vectors __vectors) ())
    )
   (send-super :init)
   (setq _vectors __vectors)
   self)
  (:vectors
   (&rest __vectors)
   (if (keywordp (car __vectors))
       (send* _vectors __vectors)
     (progn
       (if __vectors (setq _vectors (car __vectors)))
       _vectors)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3[] _vectors
    (apply #'+ (send-all _vectors :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3[] _vectors
     (write-long (length _vectors) s)
     (dolist (elem _vectors)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3[] _vectors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vectors (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _vectors)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get autonomous_navigation::Vector3Array :md5sum-) "0e70d69b80b6619295db7fb48376314f")
(setf (get autonomous_navigation::Vector3Array :datatype-) "autonomous_navigation/Vector3Array")
(setf (get autonomous_navigation::Vector3Array :definition-)
      "# Vector3Array.msg
# Un mensaje que contiene un arreglo de geometry_msgs/Vector3

geometry_msgs/Vector3[] vectors
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :autonomous_navigation/Vector3Array "0e70d69b80b6619295db7fb48376314f")


