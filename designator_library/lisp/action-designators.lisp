;;; Copyright (c) 2013, 2016 Gheorghe Lisca <lisca@cs.uni-bremen.de>
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

(in-package :designator-library)

(defparameter grasping-designator
  (make-instance 'cram-designators:designator
    :description `((:action-pre-grasping-position nil)
                   (:action-grasping-position nil))))

(defparameter front-grasping-designator
  (make-instance 'cram-designators:designator
    :description `((:pre-grasping-position :object-front-frame)
                   (:grasping-position :object-front-grasping-frame))))

(defparameter side-grasping-designator
  (make-instance 'cram-designators:designator
    :description `((:pre-grasping-position :object-side-frame)
                   (:grasping-position :object-side-grasping-frame))))

(defparameter top-grasping-designator
  (make-instance 'cram-designators:designator
    :description `((:pre-grasping-position :object-top-frame)
                   (:grasping-position :object-top-grasping-frame))))

(defparameter left-grasping-designator
  (make-instance 'cram-designators:designator
    :description `((:pre-grasping-position :object-left-frame)
                   (:grasping-position :object-left-grasping-frame))))

(defparameter right-grasping-designator
  (make-instance 'cram-designators:designator
    :description `((:pre-grasping-position :object-right-frame)
                   (:grasping-position :object-right-grasping-frame))))

(defparameter dual-arm-left-right-grasping-designator
  (make-instance 'cram-designators:designator
    :description `((:left-pre-grasping-position :object-left-frame)
                   (:left-grasping-position :object-left-grasping-frame)
                   (:right-pre-grasping-position :object-right-frame)
                   (:right-grasping-position :object-right-grasping-frame))))

(defparameter dual-arm-holding-designator
  (make-instance 'cram-designators:designator
    :description `((:left-grasping-position :object-part-pushed-frame)
                   (:right-grasping-position :object-side-grasping-frame))))

