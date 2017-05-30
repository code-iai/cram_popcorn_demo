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

(defparameter *scooping-frame-name*
  "scooping_frame")

(defparameter *scooping-over-frame-name*
  "scooping_over_frame")

(defparameter *scooping-0-frame-name*
  "scooping_0_frame")

(defparameter *scooping-1-frame-name*
  "scooping_1_frame")

(defparameter *scooping-2-frame-name*
  "scooping_2_frame")


(defparameter *scooping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "map"
    :child-frame-id *scooping-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *scooping-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *scooping-frame-name*
    :child-frame-id *scooping-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.020 0.059 0.17)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.329 -0.203 0.319 0.865))))

(defparameter *scooping-0-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *scooping-frame-name*
    :child-frame-id *scooping-0-frame-name*
    :translation (cl-transforms:make-3d-vector -0.030 0.000 0.000)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.276 -0.073 0.314 0.905))))

(defparameter *scooping-1-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *scooping-frame-name*
    :child-frame-id *scooping-1-frame-name*
    :translation (cl-transforms:make-3d-vector 0.030 -0.020 0.000)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.276 -0.073 0.314 0.905))))

(defparameter *scooping-2-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *scooping-frame-name*
    :child-frame-id *scooping-2-frame-name*
    :translation (cl-transforms:make-3d-vector 0.040 0.000 0.20)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.567 -0.157 0.304 0.749))))

(defun publish-scooping-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *scooping-frame*
   *scooping-over-frame*
   
   *scooping-0-frame*
   *scooping-1-frame*
   *scooping-2-frame* ))
