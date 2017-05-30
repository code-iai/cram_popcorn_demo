;;; Copyright (c) 2016 Gheorghe Lisca <lisca@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Gheorghe Lisca nor the names of his
;;;       partners may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :designator-library)

(defparameter *pr2-base-footprint*
  "base_footprint")


(defparameter *in-pr2-left-gripper-tool-frame-name*
  "in_pr2_left_gripper_tool_frame")

(defparameter *in-pr2-right-gripper-tool-frame-name*
  "in_pr2_right_gripper_tool_frame")


(defparameter *pr2-ready-arm-left-frame-name*
  "pr2_ready_arm_left_frame")

(defparameter *pr2-ready-arm-right-frame-name*
  "pr2_ready_arm_right_frame")


(defparameter *pr2-ready-arm-left-at-chest-frame-name*
  "pr2_ready_arm_left_at_chest_frame")

(defparameter *pr2-ready-arm-right-at-chest-frame-name*
  "pr2_ready_arm_right_at_chest_frame")


(defparameter *pr2-ready-arm-left-at-chest-crossed-low-frame-name*
  "pr2_ready_arm_left_at_chest_crossed_low_frame")

(defparameter *pr2-ready-arm-right-at-chest-crossed-low-frame-name*
  "pr2_ready_arm_right_at_chest_crossed_low_frame")


(defparameter *pr2-ready-arm-left-at-chest-low-frame-name*
  "pr2_ready_arm_left_at_chest_low_frame")

(defparameter *pr2-ready-arm-right-at-chest-low-frame-name*
  "pr2_ready_arm_right_at_chest_low_frame")

;;; =======================================================

(defparameter *in-pr2-left-gripper-tool-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "l_gripper_tool_frame"
    :child-frame-id *in-pr2-left-gripper-tool-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *in-pr2-right-gripper-tool-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "r_gripper_tool_frame"
    :child-frame-id *in-pr2-right-gripper-tool-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

;;; =======================================================

(defparameter *pr2-ready-arm-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "torso_lift_link"
    :child-frame-id *pr2-ready-arm-left-frame-name*
    :translation (cl-transforms:make-3d-vector
                  0.100 0.800 0.250)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *pr2-ready-arm-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "torso_lift_link"
    :child-frame-id *pr2-ready-arm-right-frame-name*
    :translation (cl-transforms:make-3d-vector
                  0.100 -0.800 0.250)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *pr2-ready-arm-left-at-chest-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "torso_lift_link"
    :child-frame-id *pr2-ready-arm-left-at-chest-frame-name*
    :translation (cl-transforms:make-3d-vector
                  0.350 0.300 0.000)
    :rotation (cl-transforms:euler->quaternion
               ;; :ay (/ 3.14 6.0)
               ;; :ax (/ 3.14 3.0)
               :az (/ -3.14 2.0))))

(defparameter *pr2-ready-arm-right-at-chest-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "torso_lift_link"
    :child-frame-id *pr2-ready-arm-right-at-chest-frame-name*
    :translation (cl-transforms:make-3d-vector
                  0.350 -0.300 0.000)
    :rotation (cl-transforms:euler->quaternion
               ;; :ay (/ 3.14 6.0)
               ;; :ax (/ 3.14 3.0)
               :az (/ 3.14 2.0))))

(defparameter *pr2-ready-arm-left-at-chest-crossed-low-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "torso_lift_link"
    :child-frame-id *pr2-ready-arm-left-at-chest-crossed-low-frame-name*
    :translation (cl-transforms:make-3d-vector
                  0.400 -0.150 -0.200)
    :rotation (cl-transforms:euler->quaternion
               ;; :ay (/ 3.14 6.0)
               ;; :ax (/ 3.14 3.0)
               :az (/ -3.14 2.0))))

(defparameter *pr2-ready-arm-right-at-chest-crossed-low-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "torso_lift_link"
    :child-frame-id *pr2-ready-arm-right-at-chest-crossed-low-frame-name*
    :translation (cl-transforms:make-3d-vector
                  0.400 0.150 -0.200)
    :rotation (cl-transforms:euler->quaternion
               ;; :ay (/ 3.14 6.0)
               ;; :ax (/ 3.14 3.0)
               :az (/ 3.14 2.0))))

(defparameter *pr2-ready-arm-left-at-chest-low-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "torso_lift_link"
    :child-frame-id *pr2-ready-arm-left-at-chest-low-frame-name*
    :translation (cl-transforms:make-3d-vector
                  0.350 0.100 -0.200)
    :rotation (cl-transforms:euler->quaternion
               ;; :ay (/ 3.14 6.0)
               ;; :ax (/ 3.14 3.0)
               :az (/ -3.14 2.0))))

(defparameter *pr2-ready-arm-right-at-chest-low-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "torso_lift_link"
    :child-frame-id *pr2-ready-arm-right-at-chest-low-frame-name*
    :translation (cl-transforms:make-3d-vector
                  0.350 -0.100 -0.200)
    :rotation (cl-transforms:euler->quaternion
               ;; :ay (/ 3.14 6.0)
               ;; :ax (/ 3.14 3.0)
               :az (/ 3.14 2.0))))

;;; =======================================================

(defparameter *in-pr2-left-gripper-tool-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "l_gripper_tool_frame"
    :child-frame-id *in-pr2-left-gripper-tool-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *in-pr2-right-gripper-tool-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "r_gripper_tool_frame"
    :child-frame-id *in-pr2-right-gripper-tool-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

;;; =======================================================

(defun broadcast-pr2-arms-postures-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *pr2-ready-arm-left-frame*
   *pr2-ready-arm-right-frame*

   *pr2-ready-arm-left-at-chest-frame*
   *pr2-ready-arm-right-at-chest-frame*

   *pr2-ready-arm-left-at-chest-low-frame*
   *pr2-ready-arm-right-at-chest-low-frame*

   *pr2-ready-arm-left-at-chest-crossed-low-frame*
   *pr2-ready-arm-right-at-chest-crossed-low-frame*

   *in-pr2-left-gripper-tool-frame*
   *in-pr2-right-gripper-tool-frame*))

;;; =======================================================

;; todo(lisca): publish these frames only when necessary!
(roslisp-utilities:register-ros-init-function broadcast-pr2-arms-postures-frames)

;;; =======================================================
