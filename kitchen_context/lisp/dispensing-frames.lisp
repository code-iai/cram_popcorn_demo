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

(defparameter *dispensing-frame-name*
  "dispensing_frame")

(defparameter *dispensing-over-frame-name*
  "dispensing_over_frame")

(defparameter *dispensing-0-frame-name*
  "dispensing_0_frame")

(defparameter *dispensing-1-frame-name*
  "dispensing_1_frame")

(defparameter *dispensing-2-frame-name*
  "dispensing_2_frame")


(defparameter *dispensing-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "map"
    :child-frame-id *dispensing-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *dispensing-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *dispensing-frame-name*
    :child-frame-id *dispensing-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.25)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.610 -0.123 0.111 0.775))))

(defparameter *dispensing-0-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *dispensing-frame-name*
    :child-frame-id *dispensing-0-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.563 -0.120 0.100 0.811))))

(defparameter *dispensing-1-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *dispensing-frame-name*
    :child-frame-id *dispensing-1-frame-name*
    :translation (cl-transforms:make-3d-vector -0.020 0.007 0.10)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.325 -0.465 -0.507 0.648))))

(defparameter *dispensing-2-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *dispensing-frame-name*
    :child-frame-id *dispensing-2-frame-name*
    :translation (cl-transforms:make-3d-vector 0.020 0.002 0.10)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion 0.201 0.522 0.682 -0.472))))

(defun publish-dispensing-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *dispensing-frame*

   *dispensing-over-frame*

   *dispensing-0-frame*
   *dispensing-1-frame*
   *dispensing-2-frame*))
