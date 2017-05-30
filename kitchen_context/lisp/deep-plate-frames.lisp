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

(defparameter *deep-plate-initial-location-frame-name*
  "deep_plate_initial_location_frame")

(defparameter *deep-plate-frame-name*
  "deep_plate_frame")

(defparameter *deep-plate-over-frame-name*
  "deep_plate_over_frame")

(defparameter *deep-plate-over-salting-frame-name*
  "deep_plate_over_salting_frame")

(defparameter *deep-plate-pregrasping-frame-name*
  "deep_plate_pregrasping_frame")

(defparameter *deep-plate-grasping-frame-name*
  "deep_plate_grasping_frame")

(defparameter *deep-plate-pouring-in-1-frame-name*
  "deep_plate_pouring_in_1_frame")

(defparameter *deep-plate-pouring-in-2-frame-name*
  "deep_plate_pouring_in_2_frame")

(defparameter *deep-plate-pouring-in-3-frame-name*
  "deep_plate_pouring_in_3_frame")

(defparameter *deep-plate-pouring-in-4-frame-name*
  "deep_plate_pouring_in_4_frame")

(defparameter *deep-plate-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-drawer-left-frame-name*
    :child-frame-id *deep-plate-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector -0.03 -0.03 0.02)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *deep-plate-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-initial-location-frame-name*
    :child-frame-id *deep-plate-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.00)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *deep-plate-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-frame-name*
    :child-frame-id *deep-plate-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.25)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 1)
               ;; :ay (/ 3.14 2)
               ;; :az (/ -3.14 4)
               )))

(defparameter *deep-plate-over-salting-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-frame-name*
    :child-frame-id *deep-plate-over-salting-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.30)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ 3.14 1)
               ;; :ay (/ 3.14 2)
               :az (/ -3.14 1)
               )))

(defparameter *deep-plate-pregrasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-frame-name*
    :child-frame-id *deep-plate-pregrasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.095 0.00 0.13)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ 3.14 5)
               :ay (/ 3.14 3)
               :az (/ -3.14 4))))

(defparameter *deep-plate-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-frame-name*
    :child-frame-id *deep-plate-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.0900 0.0130 0.0144)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.6267 -0.3339 0.2745 -0.6482))))

(defparameter *deep-plate-pouring-in-1-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-frame-name*
    :child-frame-id *deep-plate-pouring-in-1-frame-name*
    :translation (cl-transforms:make-3d-vector -0.10 0.00 0.25)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 2)
               :ay (/ 3.14 5)
               ;; :az (/ -3.14 4)
               )))

(defparameter *deep-plate-pouring-in-2-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-frame-name*
    :child-frame-id *deep-plate-pouring-in-2-frame-name*
    :translation (cl-transforms:make-3d-vector -0.10 0.00 0.25)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 2)
               :ay (/ 3.14 4)
               ;; :az (/ -3.14 4)
               )))

(defparameter *deep-plate-pouring-in-3-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-frame-name*
    :child-frame-id *deep-plate-pouring-in-3-frame-name*
    :translation (cl-transforms:make-3d-vector -0.10 0.00 0.25)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 2)
               :ay (/ 3.14 3)
               ;; :az (/ -3.14 4)
               )))

(defparameter *deep-plate-pouring-in-4-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *deep-plate-frame-name*
    :child-frame-id *deep-plate-pouring-in-4-frame-name*
    :translation (cl-transforms:make-3d-vector -0.10 0.00 0.25)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 2)
               :ay (* 3.14 0.625)
               ;; :az (/ -3.14 4)
               )))

(defun publish-deep-plate-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *deep-plate-initial-location-frame*
   *deep-plate-frame*
   *deep-plate-over-frame*
   *deep-plate-over-salting-frame*
   *deep-plate-pregrasping-frame*
   *deep-plate-grasping-frame*
   *deep-plate-pouring-in-1-frame*
   *deep-plate-pouring-in-2-frame*
   *deep-plate-pouring-in-3-frame*
   *deep-plate-pouring-in-4-frame*
   ))
