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

(defparameter *spoon-flatten-initial-location-frame-name*
  "spoon_flatten_initial_location_frame")

(defparameter *spoon-flatten-over-initial-location-frame-name*
  "spoon_flatten_over_initial_location_frame")


(defparameter *spoon-flatten-frame-name*
  "spoon_flatten_frame")


(defparameter *spoon-flatten-grasping-frame-name*
  "spoon_flatten_grasping_frame")

(defparameter *spoon-flatten-pre-grasping-frame-name*
  "spoon_flatten_pre_grasping_frame")


(defparameter *spoon-flatten-end-effector-frame-name*
  "spoon_flatten_end-effector_frame")


(defparameter *spoon-flatten-initial-location-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tool-holder-rack-1-frame-name*
    :child-frame-id *spoon-flatten-initial-location-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:euler->quaternion)))

(defparameter *spoon-flatten-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spoon-flatten-initial-location-frame-name*
    :child-frame-id *spoon-flatten-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *spoon-flatten-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spoon-flatten-frame-name*
    :child-frame-id *spoon-flatten-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.01 0.005 -0.01)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *spoon-flatten-pre-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spoon-flatten-grasping-frame-name*
    :child-frame-id *spoon-flatten-pre-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 -0.07)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *spoon-flatten-end-effector-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spoon-flatten-frame-name*
    :child-frame-id *spoon-flatten-end-effector-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -0.008 -0.220)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defun publish-spoon-flatten-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *spoon-flatten-initial-location-frame*

   *spoon-flatten-frame*
   *spoon-flatten-grasping-frame*
   *spoon-flatten-pre-grasping-frame*
   *spoon-flatten-end-effector-frame*))
