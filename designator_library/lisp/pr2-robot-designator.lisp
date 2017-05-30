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

(defparameter in-pr2-left-gripper-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*in-pr2-left-gripper-tool-frame*))))

(defparameter in-pr2-right-gripper-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*in-pr2-right-gripper-tool-frame*))))


(defparameter pr2-ready-arm-left-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*pr2-ready-arm-left-frame*))))

(defparameter pr2-ready-arm-right-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*pr2-ready-arm-right-frame*))))


(defparameter pr2-ready-arm-left-at-chest-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*pr2-ready-arm-left-at-chest-frame*))))

(defparameter pr2-ready-arm-right-at-chest-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*pr2-ready-arm-right-at-chest-frame*))))


(defparameter pr2-ready-arm-left-at-chest-low-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*pr2-ready-arm-left-at-chest-low-frame*))))

(defparameter pr2-ready-arm-right-at-chest-low-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*pr2-ready-arm-right-at-chest-low-frame*))))


(defparameter pr2-ready-arm-left-at-chest-crossed-low-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*pr2-ready-arm-left-at-chest-crossed-low-frame*))))

(defparameter pr2-ready-arm-right-at-chest-crossed-low-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*pr2-ready-arm-right-at-chest-crossed-low-frame*))))

;; ======================================================= ;;

(defparameter *pr2-robot-designator*
  (make-instance 'cram-designators:designator
    :description `((:robot-base-interface nil)
                   (:robot-torso-interface nil)
                   (:robot-left-arm-interface nil)
                   (:robot-right-arm-interface nil)
                   (:robot-left-gripper-interface nil)
                   (:robot-right-gripper-interface nil)
                   (:robot-neck-interface nil)

                   (:robot-navigation-inteface nil)
                   (:robot-manipulation-inteface nil)
                   (:robot-perception-interface nil)
                   (:robot-knowledge-interface nil)
                   
                   (:in-left-gripper-frame ,*in-pr2-left-gripper-tool-frame*)
                   (:in-right-gripper-frame ,*in-pr2-right-gripper-tool-frame*)

                   (:ready-arm-left-frame ,*pr2-ready-arm-left-frame*)
                   (:ready-arm-right-frame ,*pr2-ready-arm-right-frame*)

                   (:ready-arm-left-at-chest-frame ,*pr2-ready-arm-left-at-chest-frame*)
                   (:ready-arm-right-at-chest-frame ,*pr2-ready-arm-right-at-chest-frame*)

                   (:ready-arm-left-at-chest-low-frame ,*pr2-ready-arm-left-at-chest-low-frame*)
                   (:ready-arm-right-at-chest-low-frame ,*pr2-ready-arm-right-at-chest-low-frame*)

                   (:ready-arm-left-at-chest-crossed-low-frame ,*pr2-ready-arm-left-at-chest-crossed-low-frame*)
                   (:ready-arm-right-at-chest-crossed-low-frame ,*pr2-ready-arm-right-at-chest-crossed-low-frame*))))

;; ======================================================= ;;
