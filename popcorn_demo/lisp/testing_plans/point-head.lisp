;;; Testing the robot's head moving


(in-package :popcorn-demo)

;; Define some frames for robot head moving 

(defparameter *left-point-frame-name*
  "left-point-frame")

(defparameter *right-point-frame-name*
  "right-point-frame")

(defparameter *left-point-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "map"
    :child-frame-id *left-point-frame-name*
    :translation (cl-transforms:make-3d-vector 10.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *right-point-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "map" 
    :child-frame-id *right-point-frame-name*
    :translation (cl-transforms:make-3d-vector -10.0 0.1 0.0)
    :rotation (cl-transforms:make-identity-rotation)))


(defun publish-head-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly
    *left-point-frame*
    *right-point-frame*))

;;Define designators for every frame

(defparameter left-point-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*left-point-frame*))))

(defparameter right-point-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*right-point-frame*))))


;;Define the function for navigation

(defun move-head (location-designator)
  (plan-library::sighted location-designator)

  (roslisp:ros-info 'popcorn-demo "Navigation plan tested."))



;;execute all tests
(defun head-moving  ()

  (roslisp-utilities:startup-ros)
  (roslisp:ros-info 'popcorn-demo "Ros-node started.")

  (popcorn-demo::initialize-popcorn-demo)
  (roslisp:ros-info 'popcorn-demo "Popcorn-demo initialized.")

  (sleep 1)
  (popcorn-demo::publish-head-frames)
  (roslisp:ros-info 'popcorn-demo "Frames published.")
  
  (move-head left-point-location-designator)
  (move-head right-point-location-designator)
   (move-head left-point-location-designator)
  (roslisp:ros-info 'popcorn-demo "All tests finished."))
