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

(defparameter *tool-holder-initial-location-frame-name*
  "tool_holder_initial_location_frame")


(defparameter *tool-holder-frame-name*
  "tool_holder_frame")


(defparameter *tool-holder-over-frame-name*
  "tool_holder_over_frame")

(defparameter *tool-holder-front-frame-name*
  "tool_holder_front_frame")


(defparameter *tool-holder-rack-0-frame-name*
  "tool_holder_rack_0_frame")

(defparameter *tool-holder-rack-1-frame-name*
  "tool_holder_rack_1_frame")

(defparameter *tool-holder-rack-2-frame-name*
  "tool_holder_rack_2_frame")

(defparameter *tool-holder-rack-3-frame-name*
  "tool_holder_rack_3_frame")


(defparameter *tool-holder-over-rack-0-frame-name*
  "tool_holder_over_rack_0_frame")

(defparameter *tool-holder-over-rack-1-frame-name*
  "tool_holder_over_rack_1_frame")

(defparameter *tool-holder-over-rack-2-frame-name*
  "tool_holder_over_rack_2_frame")

(defparameter *tool-holder-over-rack-3-frame-name*
  "tool_holder_over_rack_3_frame")


(defparameter *tool-holder-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *kitchen-island-frame-name*
    :child-frame-id *tool-holder-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector 0.300 1.525 0.86)
    :rotation (cl-transforms:euler->quaternion)))

(defparameter *tool-holder-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tool-holder-initial-location-frame-name*
    :child-frame-id *tool-holder-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *tool-holder-rack-0-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tool-holder-frame-name*
    :child-frame-id *tool-holder-rack-0-frame-name*
    :translation (cl-transforms:make-3d-vector -0.175 -0.025 0.26)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *tool-holder-over-rack-0-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tool-holder-rack-0-frame-name*
    :child-frame-id *tool-holder-over-rack-0-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -0.10 0.10)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *tool-holder-rack-1-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tool-holder-frame-name*
    :child-frame-id *tool-holder-rack-1-frame-name*
    :translation (cl-transforms:make-3d-vector -0.055 -0.025 0.26)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *tool-holder-over-rack-1-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *tool-holder-rack-1-frame-name*
    :child-frame-id *tool-holder-over-rack-1-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -0.10 0.10)
    :rotation (cl-transforms:make-identity-rotation)))

(defun publish-tool-holder-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *tool-holder-initial-location-frame*

   *tool-holder-frame*
   
   *tool-holder-rack-0-frame*
   *tool-holder-over-rack-0-frame*
   *tool-holder-rack-1-frame*
   *tool-holder-over-rack-1-frame*))
