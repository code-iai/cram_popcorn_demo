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

(in-package :kitchen-context)

(defparameter *spoon-rounded-initial-location-frame-name*
  "spoon_rounded_initial_location_frame")

(defparameter *spoon-rounded-over-initial-location-frame-name*
  "spoon_rounded_over_initial_location_frame")


(defparameter *spoon-rounded-frame-name*
  "spoon_rounded_frame")


(defparameter *spoon-rounded-grasping-frame-name*
  "spoon_rounded_grasping_frame")

(defparameter *spoon-rounded-pre-grasping-frame-name*
  "spoon_rounded_pre_grasping_frame")


(defparameter *spoon-rounded-end-effector-frame-name*
  "spoon_rounded_end_effector_frame")


(defparameter *spoon-rounded-initial-location-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tool-holder-rack-0-frame-name*
    :child-frame-id *spoon-rounded-initial-location-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:euler->quaternion)))

(defparameter *spoon-rounded-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spoon-rounded-initial-location-frame-name*
    :child-frame-id *spoon-rounded-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *spoon-rounded-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spoon-rounded-frame-name*
    :child-frame-id *spoon-rounded-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.01 0.005 -0.01)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *spoon-rounded-pre-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spoon-rounded-grasping-frame-name*
    :child-frame-id *spoon-rounded-pre-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 -0.07)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *spoon-rounded-end-effector-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spoon-rounded-frame-name*
    :child-frame-id *spoon-rounded-end-effector-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.01 -0.20)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defun publish-spoon-rounded-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *spoon-rounded-initial-location-frame*

   *spoon-rounded-frame*
   *spoon-rounded-grasping-frame*
   *spoon-rounded-pre-grasping-frame*
   *spoon-rounded-end-effector-frame*))
