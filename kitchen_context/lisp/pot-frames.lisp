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

(defparameter *pot-initial-location-frame-name*
  "pot_initial_location_frame")

(defparameter *pot-frame-name*
  "pot_frame")

(defparameter *pot-over-frame-name*
  "pot_over_frame")

(defparameter *pot-pouring-in-frame-name*
  "pot_pouring_in_frame")

(defparameter *pot-pregrasping-left-frame-name*
  "pot_pregrasping_left_frame")

(defparameter *pot-pregrasping-right-frame-name*
  "pot_pregrasping_right_frame")

(defparameter *pot-grasping-left-frame-name*
  "pot_grasping_left_frame")

(defparameter *pot-grasping-right-frame-name*
  "pot_grasping_right_frame")

(defparameter *pot-top-frame-name*
  "pot_top_frame")

(defparameter *pot-shaking-1-frame-name*
  "pot_shaking_1_frame")

(defparameter *pot-shaking-2-frame-name*
  "pot_shaking_2_frame")

(defparameter *pot-shaking-3-frame-name*
  "pot_shaking_3_frame")


(defparameter *pot-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *pot-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector -0.15 -0.6 -0.02)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *pot-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-initial-location-frame-name*
    :child-frame-id *pot-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 2.0)
               ;; :ay (/ 3.14 2.0)
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *pot-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -0.05 0.25)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *pot-pouring-in-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-pouring-in-frame-name*
    :translation (cl-transforms:make-3d-vector 0.02 -0.08 0.25)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ -3.14 1.7)
               ;; :ay (/ 3.14 2.0)
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *pot-pregrasping-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-pregrasping-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.20 0.09)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ -3.14 2)
               :ay (/ 3.14 1)
               :az (/ -3.14 0.85))))

(defparameter *pot-pregrasping-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-pregrasping-right-frame-name*
    :translation (cl-transforms:make-3d-vector  0.0 -0.20 0.09)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ 3.14 2)
               :ay (/ 3.14 1)
               :az (/ -3.14 1.2)
               )))

(defparameter *pot-grasping-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-grasping-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.137 0.08)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ -3.14 2)
               :ay (/ 3.14 1)
               :az (/ -3.14 0.85))))

(defparameter *pot-grasping-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-grasping-right-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -0.13 0.08)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ 3.14 2)
               :ay (/ 3.14 1)
               :az (/ -3.14 1.2))))

(defparameter *pot-top-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-top-frame-name*
    :translation (cl-transforms:make-3d-vector 0.00 0.000 0.100)
    :rotation (cl-transforms:make-identity-rotation)))


(defparameter *pot-shaking-1-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-shaking-1-frame-name*
    :translation (cl-transforms:make-3d-vector 0.000 0.000 0.000)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 4)
               :ay (/ 3.14 10)
               ;; :az (/ -3.14 4)
               )))

(defparameter *pot-shaking-2-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-shaking-2-frame-name*
    :translation (cl-transforms:make-3d-vector 0.000 0.000 0.000)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ 3.14 10)
               ;; :ay (/ 3.14 4)
               ;; :az (/ -3.14 4)
               )))

(defparameter *pot-shaking-3-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *pot-frame-name*
    :child-frame-id *pot-shaking-3-frame-name*
    :translation (cl-transforms:make-3d-vector 0.000 0.000 0.000)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ -3.14 10)
               ;; :ay (/ 3.14 4)
               ;; :az (/ -3.14 4)
               )))

(defun publish-pot-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *pot-initial-location-frame*
   *pot-frame*
   *pot-over-frame*
   *pot-pouring-in-frame*
   *pot-pregrasping-left-frame*
   *pot-pregrasping-right-frame*
   *pot-grasping-left-frame*
   *pot-grasping-right-frame*
   *pot-top-frame*
   *pot-shaking-1-frame*
   *pot-shaking-2-frame*
   *pot-shaking-3-frame*
   ))
