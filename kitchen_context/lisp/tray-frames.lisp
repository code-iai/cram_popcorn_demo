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

(defparameter *tray-initial-location-frame-name*
  "tray_initial_location_frame")


(defparameter *tray-frame-name*
  "tray_frame")


(defparameter *tray-over-frame-name*
  "tray_over_frame")

(defparameter *tray-away-frame-name*
  "tray_away_frame")

(defparameter *tray-zone-frame-name*
  "tray_zone_frame")

(defparameter *tray-zone-0-frame-name*
  "tray_zone_0_frame")

(defparameter *tray-zone-1-frame-name*
  "tray_zone_1_frame")

(defparameter *tray-zone-2-frame-name*
  "tray_zone_2_frame")

(defparameter *tray-zone-3-frame-name*
  "tray_zone_3_frame")

(defparameter *tray-zone-4-frame-name*
  "tray_zone_4_frame")

(defparameter *tray-zone-5-frame-name*
  "tray_zone_5_frame")


(defparameter *tray-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *kitchen-island-frame-name*
    :child-frame-id *tray-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector 0.2 0.2 0.86)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *tray-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-initial-location-frame-name*
    :child-frame-id *tray-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.01)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *tray-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-frame-name*
    :child-frame-id *tray-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *tray-away-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-frame-name*
    :child-frame-id *tray-away-frame-name*
    :translation (cl-transforms:make-3d-vector 0.2 0.2 0.10)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.540 -0.369 0.305 0.693))))

(defparameter *tray-zone-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-frame-name*
    :child-frame-id *tray-zone-frame-name*
    :translation (cl-transforms:make-3d-vector 0.03 -0.03 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *tray-zone-0-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-zone-frame-name*
    :child-frame-id *tray-zone-0-frame-name*
    :translation (cl-transforms:make-3d-vector -0.04 -0.05 0.0)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 -2.25)
               ;; :ay 3.14
               ;; :az (/ 3.14 8.0)
               )))

(defparameter *tray-zone-1-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-zone-frame-name*
    :child-frame-id *tray-zone-1-frame-name*
    :translation (cl-transforms:make-3d-vector -0.04 0.0 0.0)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 -2.25)
               ;; :ay 3.14
               ;; :az (/ 3.14 8.0)
               )))

(defparameter *tray-zone-2-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-zone-frame-name*
    :child-frame-id *tray-zone-2-frame-name*
    :translation (cl-transforms:make-3d-vector -0.04 0.05 0.0)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 -2.25)
               ;; :ay 3.14
               ;; :az (/ 3.14 8.0)
               )))

(defparameter *tray-zone-3-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-zone-frame-name*
    :child-frame-id *tray-zone-3-frame-name*
    :translation (cl-transforms:make-3d-vector 0.04 0.05 0.0)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 -2.25)
               ;; :ay 3.14
               ;; :az (/ -3.14 10.0)
               )))

(defparameter *tray-zone-4-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-zone-frame-name*
    :child-frame-id *tray-zone-4-frame-name*
    :translation (cl-transforms:make-3d-vector 0.04 0.0 0.0)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 -2.25)
               ;; :ay 3.14
               ;; :az (/ -3.14 8.0)
               )))

(defparameter *tray-zone-5-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tray-zone-frame-name*
    :child-frame-id *tray-zone-5-frame-name*
    :translation (cl-transforms:make-3d-vector 0.04 -0.05 0.0)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 -2.25)
               ;; :ay 3.14
               ;; :az (/ -3.14 6.0)
               )))


(defun publish-tray-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *tray-initial-location-frame*

   *tray-frame*
   *tray-over-frame*
   *tray-away-frame*

   *tray-zone-frame*
   *tray-zone-0-frame*
   *tray-zone-1-frame*
   *tray-zone-2-frame*
   *tray-zone-3-frame*
   *tray-zone-4-frame*
   *tray-zone-5-frame*))
