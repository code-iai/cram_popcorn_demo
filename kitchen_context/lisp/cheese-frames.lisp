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

(defparameter *cheese-frame-name*
  "cheese_frame")

(defparameter *cheese-over-frame-name*
  "cheese_over_frame")


(defparameter *cheese-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *yellow-bowl-frame-name*
    :child-frame-id *cheese-frame-name*
    :translation (cl-transforms:make-identity-vector)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *cheese-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *cheese-frame-name*
    :child-frame-id *cheese-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.15)
    :rotation (cl-transforms:make-identity-rotation)))

(defun publish-cheese-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *cheese-frame*
   *cheese-over-frame*))
