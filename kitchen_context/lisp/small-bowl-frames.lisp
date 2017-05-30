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
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE#
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :kitchen-context)

(defparameter *small-bowl-initial-location-frame-name*
  "small_bowl initial_location_frame")

(defparameter *small-bowl-frame-name*
  "small_bowl_frame")

(defparameter *small-bowl-over-frame-name*
  "small_bowl_over_frame")

(defparameter *small-bowl-pregrasping-frame-name*
  "small_bowl_pregrasping_frame")

(defparameter *small-bowl-grasping-frame-name*
  "small_bowl_grasping_frame")


(defparameter *small-bowl-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-drawer-right-frame-name*
    :child-frame-id *small-bowl-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -0.09 0.02)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *small-bowl-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *small-bowl-initial-location-frame-name*
    :child-frame-id *small-bowl-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.00)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *small-bowl-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *small-bowl-frame-name*
    :child-frame-id *small-bowl-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *small-bowl-pregrasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *small-bowl-frame-name*
    :child-frame-id *small-bowl-pregrasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.14 -0.01 0.1)
    :rotation(cl-transforms:euler->quaternion
                :ax (/ 3.14 4)
                :ay (/ 3.14 3.5)
                ; :az (/ -3.14 1.5)
              )))

(defparameter *small-bowl-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *small-bowl-frame-name*
    :child-frame-id *small-bowl-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.05 -0.03 0.03)
    :rotation(cl-transforms:euler->quaternion
                :ax (/ 3.14 4)
                :ay (/ 3.14 3.5)
                ; :az (/ -3.14 1.5)
              )))


(defun publish-small-bowl-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *small-bowl-initial-location-frame*
   *small-bowl-frame*
   *small-bowl-over-frame*
   *small-bowl-pregrasping-frame*
   *small-bowl-grasping-frame*))
