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

(defparameter *boxy-base-footprint*
  "base_footprint")


(defparameter *in-boxy-left-gripper-tool-frame-name*
  "in_boxy_left_gripper_tool_frame")

(defparameter *in-boxy-right-gripper-tool-frame-name*
  "in_boxy_right_gripper_tool_frame")


(defparameter *boxy-ready-arm-left-frame-name*
  "boxy_ready_arm_left_frame")

(defparameter *boxy-ready-arm-right-frame-name*
  "boxy_ready_arm_right_frame")


(defparameter *boxy-away-arm-left-frame-name*
  "boxy_away_arm_left_frame")

(defparameter *boxy-away-arm-right-frame-name*
  "boxy_away_arm_right_frame")


(defparameter *boxy-ready-arm-left-at-chest-frame-name*
  "boxy_ready_arm_left_at_chest_frame")

(defparameter *boxy-ready-arm-right-at-chest-frame-name*
  "boxy_ready_arm_right_at_chest_frame")


;; (defparameter *ready-arm-left-at-chest-crossed-low-frame-name*
;;   "ready_arm_left_at_chest_crossed_low_frame")

;; (defparameter *ready-arm-right-at-chest-crossed-low-frame-name*
;;   "ready_arm_right_at_chest_crossed_low_frame")


;; (defparameter *ready-arm-left-at-chest-low-frame-name*
;;   "ready_arm_left_at_chest_low_frame")

;; (defparameter *ready-arm-right-at-chest-low-frame-name*
;;   "ready_arm_right_at_chest_low_frame")

;;; =======================================================

(defparameter *in-boxy-left-gripper-tool-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "left_gripper_tool_frame"
    :child-frame-id *in-boxy-left-gripper-tool-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *in-boxy-right-gripper-tool-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "right_gripper_tool_frame"
    :child-frame-id *in-boxy-right-gripper-tool-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

;;; =======================================================

(defparameter *boxy-ready-arm-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *boxy-base-footprint*
    :child-frame-id *boxy-ready-arm-left-frame-name*
    :translation (cl-transforms:make-3d-vector 1.13 0.50 1.350)
    :rotation (cl-transforms:euler->quaternion
               :ax -3.14
               ;; :ay 
               :az (/ -3.14 2.0)
               )))

(defparameter *boxy-ready-arm-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *boxy-base-footprint*
    :child-frame-id *boxy-ready-arm-right-frame-name*
    :translation (cl-transforms:make-3d-vector 1.11 -0.50 1.350)
    :rotation (cl-transforms:euler->quaternion
               :ax 3.14
               ;; :ay 
               :az (/ 3.14 2.0)
               )))

(defparameter *boxy-away-arm-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp (+ (roslisp:ros-time) 0.5)
    :frame-id *boxy-base-footprint*
    :child-frame-id *boxy-away-arm-left-frame-name*
    :translation (cl-transforms:make-3d-vector 1.363 0.559 1.550)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.361 0.715 -0.484 0.353))))

(defparameter *boxy-away-arm-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp (+ (roslisp:ros-time) 0.5)
    :frame-id *boxy-base-footprint*
    :child-frame-id *boxy-away-arm-right-frame-name*
    :translation (cl-transforms:make-3d-vector 1.108 -0.368 1.370)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion 0.667 0.742 -0.011 0.062))))

(defparameter *boxy-ready-arm-left-at-chest-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *boxy-base-footprint*
    :child-frame-id *boxy-ready-arm-left-at-chest-frame-name*
    :translation (cl-transforms:make-3d-vector 1.300 0.400 1.400)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *boxy-ready-arm-right-at-chest-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *boxy-base-footprint*
    :child-frame-id *boxy-ready-arm-right-at-chest-frame-name*
    :translation (cl-transforms:make-3d-vector 1.300 -0.400 1.400)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

;; (defparameter *ready-arm-left-at-chest-crossed-low-frame*
;;   (make-instance
;;       'cl-transforms-stamped:transform-stamped
;;     :stamp 0.0
;;     :frame-id *base-footprint*
;;     :child-frame-id *ready-arm-left-at-chest-crossed-low-frame-name*
;;     :translation (cl-transforms:make-3d-vector 0.400 -0.150 -0.200)
;;     :rotation (cl-transforms:euler->quaternion
;;                ;; :ay (/ 3.14 6.0)
;;                ;; :ax (/ 3.14 3.0)
;;                :az (/ -3.14 2.0))))

;; (defparameter *ready-arm-right-at-chest-crossed-low-frame*
;;   (make-instance
;;       'cl-transforms-stamped:transform-stamped
;;     :stamp 0.0
;;     :frame-id *base-footprint*
;;     :child-frame-id *ready-arm-right-at-chest-crossed-low-frame-name*
;;     :translation (cl-transforms:make-3d-vector 0.400 0.150 -0.200)
;;     :rotation (cl-transforms:euler->quaternion
;;                ;; :ay (/ 3.14 6.0)
;;                ;; :ax (/ 3.14 3.0)
;;                :az (/ 3.14 2.0))))

;; (defparameter *ready-arm-left-at-chest-low-frame*
;;   (make-instance
;;       'cl-transforms-stamped:transform-stamped
;;     :stamp 0.0
;;     :frame-id *base-footprint*
;;     :child-frame-id *ready-arm-left-at-chest-low-frame-name*
;;     :translation (cl-transforms:make-3d-vector 0.350 0.100 -0.200)
;;     :rotation (cl-transforms:euler->quaternion
;;                ;; :ay (/ 3.14 6.0)
;;                ;; :ax (/ 3.14 3.0)
;;                :az (/ -3.14 2.0))))

;; (defparameter *ready-arm-right-at-chest-low-frame*
;;   (make-instance
;;       'cl-transforms-stamped:transform-stamped
;;     :stamp 0.0
;;     :frame-id *base-footprint*
;;     :child-frame-id *ready-arm-right-at-chest-low-frame-name*
;;     :translation (cl-transforms:make-3d-vector 0.350 -0.100 -0.200)
;;     :rotation (cl-transforms:euler->quaternion
;;                ;; :ay (/ 3.14 6.0)
;;                ;; :ax (/ 3.14 3.0)
;;                :az (/ 3.14 2.0))))

;;; =======================================================

(defun publish-boxy-robot-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *in-boxy-left-gripper-tool-frame*
   *in-boxy-right-gripper-tool-frame*

   *boxy-ready-arm-left-frame*
   *boxy-ready-arm-right-frame*

   *boxy-away-arm-left-frame*
   *boxy-away-arm-right-frame*
   
   *boxy-ready-arm-left-at-chest-frame*
   *boxy-ready-arm-right-at-chest-frame*

   ;; *ready-arm-left-at-chest-crossed-low-frame*
   ;; *ready-arm-right-at-chest-crossed-low-frame*

   ;; *ready-arm-left-at-chest-low-frame*
   ;; *ready-arm-right-at-chest-low-frame*

   ;; *in-robot-left-gripper-tool-frame*
   ;; *in-robot-right-gripper-tool-frame*
   ))
