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

(in-package :designator-library)

;; ======================================================= ;;

(defparameter in-boxy-left-gripper-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*in-boxy-left-gripper-tool-frame*))))

(defparameter in-boxy-right-gripper-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*in-boxy-right-gripper-tool-frame*))))


(defparameter boxy-ready-arm-right-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*boxy-ready-arm-right-frame*))))

(defparameter boxy-ready-arm-left-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*boxy-ready-arm-left-frame*))))

(defparameter boxy-ready-arm-right-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*boxy-ready-arm-right-frame*))))


(defparameter boxy-away-arm-left-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*boxy-away-arm-left-frame*))))

(defparameter boxy-away-arm-right-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*boxy-away-arm-right-frame*))))


(defparameter boxy-ready-arm-left-at-chest-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*boxy-ready-arm-left-at-chest-frame*))))

(defparameter boxy-ready-arm-right-at-chest-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*boxy-ready-arm-right-at-chest-frame*))))

;; ======================================================= ;;

(defparameter *boxy-robot-designator*
  (make-instance 'cram-designators:designator
    :description `((:in-left-gripper-frame ,*in-boxy-left-gripper-tool-frame*)
                   (:in-right-gripper-frame ,*in-boxy-right-gripper-tool-frame*)

                   (:ready-arm-left-frame ,*boxy-ready-arm-left-frame*)
                   (:ready-arm-right-frame ,*boxy-ready-arm-right-frame*)

                   (:ready-arm-left-at-chest-frame ,*boxy-ready-arm-left-at-chest-frame*)
                   (:ready-arm-right-at-chest-frame ,*boxy-ready-arm-right-at-chest-frame*))))

;; ======================================================= ;;
