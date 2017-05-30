;;; Copyright (c) 2017, Mihaela Popescu <popescu@uni-bremen.de>
;;; All rights reserved.

(in-package :popcorn-demo)

;;define frames
(defparameter *stove-front-frame-name*
  "stove_front_frame")

(defparameter *stove-left-frame-name*
  "stove_left_frame")

(defparameter *stove-over-front-frame-name*
  "stove_over_front_frame")

(defparameter *stove-over-left-frame-name*
  "stove_over_left_frame")

(defparameter *stove-front-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "map"
    :child-frame-id *stove-front-frame-name*
    :translation (cl-transforms:make-3d-vector -1.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-left-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-front-frame-name*
    :child-frame-id *stove-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 2.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-over-front-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-front-frame-name*
    :child-frame-id *stove-over-front-frame-name*
    :translation (cl-transforms:make-3d-vector 0.2 -0.7 1.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-over-left-frame*
  (make-instance 'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id  *stove-left-frame-name*
    :child-frame-id *stove-over-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.2 0.7 1.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defun publish-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly
    *stove-front-frame*
    *stove-left-frame*
    *stove-over-front-frame*
    *stove-over-left-frame*))


;;define designators
(defparameter stove-front-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-front-frame*))))

(defparameter stove-left-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-left-frame*))))

(defparameter stove-over-front-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-over-front-frame*))))

(defparameter stove-over-left-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-over-left-frame*))))


;;define tests for each abstract test

(defun test-navigation (location-designator)
  (plan-library::navigate-at-location location-designator)
  (roslisp:ros-info 'popcorn-demo "Navigation plan tested."))

(defun test-sight (location-designator)
  (plan-library::sighted location-designator)
  (roslisp:ros-info 'popcorn-demo "Head pointing(sighting) plan tested."))

(defun test-arm-move (arm location-designator)
  (plan-library::singular-arm-move-at-location arm location-designator)
  (roslisp:ros-info 'popcorn-demo "Moving arm plan tested."))

(defun test-gripper (gripper opening effort)
  (plan-library:: singular-gripper-gripped gripper opening effort)
  (roslisp:ros-info 'popcorn-demo "Gripping plan tested."))

(defun test-torso (location-designator)
  (plan-library:: move-torso location-designator)
  (roslisp:ros-info 'popcorn-demo "Torso plan tested."))


;;execute all tests
(defun execute-tests  ()

  (roslisp-utilities:startup-ros)
  (roslisp:ros-info 'popcorn-demo "Ros-node started.")

  (popcorn-demo::initialize-popcorn-demo)
  (roslisp:ros-info 'popcorn-demo "Popcorn-demo initialized.")

  (sleep 1)
  (popcorn-demo::publish-frames)
  (roslisp:ros-info 'popcorn-demo "Frames published.")

  (test-torso kitchen-context::joint-position-designator)
  (test-sight stove-front-location-designator)
  (test-navigation stove-front-location-designator)
  (test-arm-move :right stove-over-front-location-designator)
  (test-gripper :right  0.050 20.000)
  
  (test-sight stove-left-location-designator)
  (test-navigation stove-left-location-designator)
  (test-arm-move :left stove-over-left-location-designator)
  (test-gripper :left  0.050 20.000)
  (roslisp:ros-info 'popcorn-demo "All tests finished."))
