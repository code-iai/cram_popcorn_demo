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

(defparameter *green-bowl-initial-location-frame-name*
  "green_bowl_initial_location_frame")


(defparameter *green-bowl-frame-name*
  "green_bowl_frame")


(defparameter *green-bowl-over-frame-name*
  "green_bowl_over_frame")


(defparameter *green-bowl-scooping-0-frame-name*
  "green_bowl_scooping_0_frame")

(defparameter *green-bowl-scooping-1-frame-name*
  "green_bowl_scooping_1_frame")

(defparameter *green-bowl-scooping-2-frame-name*
  "green_bowl_scooping_2_frame")


(defparameter *green-bowl-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *kitchen-island-frame-name*
    :child-frame-id *green-bowl-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector 0.2 1.1 0.86)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *green-bowl-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *green-bowl-initial-location-frame-name*
    :child-frame-id *green-bowl-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.1)
    :rotation (cl-transforms:make-identity-rotation)))

(defun publish-green-bowl-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *green-bowl-initial-location-frame*

   *green-bowl-frame*))
