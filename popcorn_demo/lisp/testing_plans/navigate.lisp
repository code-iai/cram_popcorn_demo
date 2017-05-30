;;; Testing the robot navigation


(in-package :popcorn-demo)

;; Define some frames for robot navigation

(defparameter *stove-front-navigation-frame-name*
  "stove_front_navigation_frame")

(defparameter *stove-left-navigation-frame-name*
  "stove_left_navigation_frame")

(defparameter *stove-right-navigation-frame-name*
  "stove_right_navigation_frame")

(defparameter *stove-front-navigation-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "map"
    :child-frame-id *stove-front-navigation-frame-name*
    :translation (cl-transforms:make-3d-vector -1.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-left-navigation-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-front-frame-name*
    :child-frame-id *stove-left-navigation-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 2.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-right-navigation-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-front-frame-name*
    :child-frame-id *stove-right-navigation-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -1.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defun publish-navigation-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly
    *stove-front-navigation-frame*
    *stove-left-navigation-frame*
    *stove-right-navigation-frame*))

;;Define designators for every frame

(defparameter stove-front-navigation-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-front-navigation-frame*))))

(defparameter stove-left-navigation-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-left-navigation-frame*))))

(defparameter stove-right-navigation-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-right-navigation-frame*))))


;;Define the function for navigation

(defun navigation (location-designator)
  (plan-library::navigate-at-location location-designator)

  (roslisp:ros-info 'popcorn-demo "Navigation plan tested."))



;;execute all tests
(defun execute-test  ()

  (roslisp-utilities:startup-ros)
  (roslisp:ros-info 'popcorn-demo "Ros-node started.")

  (popcorn-demo::initialize-popcorn-demo)
  (roslisp:ros-info 'popcorn-demo "Popcorn-demo initialized.")

  (sleep 1)
  (popcorn-demo::publish-navigation-frames)
  (roslisp:ros-info 'popcorn-demo "Frames published.")
  
  (navigation stove-front-navigation-location-designator)
  (navigation stove-left-navigation-location-designator)
  (navigation stove-right-navigation-location-designator)
  (roslisp:ros-info 'popcorn-demo "All tests finished."))
