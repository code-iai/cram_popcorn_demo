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

(defparameter *lid-initial-location-frame-name*
  "lid initial_location_frame")

(defparameter *lid-frame-name*
  "lid_frame")

(defparameter *lid-over-frame-name*
  "lid_over_frame")

(defparameter *lid-pregrasping-frame-name*
  "lid_pregrasping_frame")

(defparameter *lid-grasping-frame-name*
  "lid_grasping_frame")


(defparameter *lid-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-drawer-right-frame-name*
    :child-frame-id *lid-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector -0.10 0.07 0.02)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *lid-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *lid-initial-location-frame-name*
    :child-frame-id *lid-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.00)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *lid-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *lid-frame-name*
    :child-frame-id *lid-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *lid-pregrasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *lid-frame-name*
    :child-frame-id *lid-pregrasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.03 0.0 0.06)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               :ay (/ 3.14 2.0)
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *lid-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *lid-frame-name*
    :child-frame-id *lid-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.035)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               :ay (/ 3.14 2.0)
               ;; :az (/ 3.14 2.0)
               )))


(defun publish-lid-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *lid-initial-location-frame*
   *lid-frame*
   *lid-over-frame*
   *lid-pregrasping-frame*
   *lid-grasping-frame*))
