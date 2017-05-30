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

(defparameter *spreading-circle-radius* 0.04)

(defparameter *spreading-frame-name*
  "spreading_frame")

(defparameter *spreading-over-frame-name*
  "spreading__over_frame")

(defparameter *spreading-0-frame-name*
  "spreading_0_frame")

(defparameter *spreading-1-frame-name*
  "spreading_1_frame")

(defparameter *spreading-2-frame-name*
  "spreading_2_frame")

(defparameter *spreading-3-frame-name*
  "spreading_3_frame")

(defparameter *spreading-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "map"
    :child-frame-id *spreading-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *spreading-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spreading-frame-name*
    :child-frame-id *spreading-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.15)
    :rotation (cl-transforms:euler->quaternion
               :ax (/ -3.14 2.5)
               ;; :ay (/ -3.14 6.0)
               ;; :az (/ 3.14 2.5)
               )))

;; successive spreading frames are one relative to the other
(defparameter *spreading-0-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spreading-frame-name*
    :child-frame-id *spreading-0-frame-name*
    :translation (cl-transforms:make-3d-vector *spreading-circle-radius* (- *spreading-circle-radius*) -0.015)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.482 -0.155 -0.081 0.859))))

(defparameter *spreading-1-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spreading-frame-name*
    :child-frame-id *spreading-1-frame-name*
    :translation (cl-transforms:make-3d-vector *spreading-circle-radius* *spreading-circle-radius* -0.015)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.482 -0.155 -0.081 0.859))))

(defparameter *spreading-2-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spreading-frame-name*
    :child-frame-id *spreading-2-frame-name*
    :translation (cl-transforms:make-3d-vector (- *spreading-circle-radius*) *spreading-circle-radius* -0.015)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.482 -0.155 -0.081 0.859))))

(defparameter *spreading-3-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *spreading-frame-name*
    :child-frame-id *spreading-3-frame-name*
    :translation (cl-transforms:make-3d-vector (- *spreading-circle-radius*) (- *spreading-circle-radius*) -0.015)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.482 -0.155 -0.081 0.859))))

;;
;; publish previously defined transforms
;;

(defun publish-spreading-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *spreading-frame*

   *spreading-over-frame*
   
   *spreading-0-frame*
   *spreading-1-frame*
   *spreading-2-frame*
   *spreading-3-frame*))
