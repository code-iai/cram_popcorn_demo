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

;; =======================================================

(defparameter tool-holder-rack-0-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*tool-holder-rack-0-frame*)
                   (:over-frame ,*tool-holder-over-rack-0-frame*))))

(defparameter tool-holder-rack-1-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*tool-holder-rack-1-frame*)
                   (:over-frame ,*tool-holder-over-rack-1-frame*))))

(defparameter tray-zone-0-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*tray-zone-0-frame*))))

(defparameter tray-zone-1-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*tray-zone-1-frame*))))

(defparameter tray-zone-2-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*tray-zone-2-frame*))))

(defparameter tray-zone-3-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*tray-zone-3-frame*))))

(defparameter tray-zone-4-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*tray-zone-4-frame*))))

(defparameter tray-zone-5-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*tray-zone-5-frame*))))


(defparameter scooping-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*scooping-frame*)
                   (:over ,*scooping-over-frame*)
                   (:scooping-0 ,*scooping-0-frame*)
                   (:scooping-1 ,*scooping-1-frame*)
                   (:scooping-2 ,*scooping-2-frame*))))


(defparameter dispensing-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*dispensing-frame*)
                   (:over ,*dispensing-over-frame*)
                   (:dispensing-0 ,*dispensing-0-frame*)
                   (:dispensing-1 ,*dispensing-1-frame*)
                   (:dispensing-2 ,*dispensing-2-frame*))))

(defparameter spreading-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*spreading-frame*)
                   (:over ,*spreading-over-frame*)
                   (:spreading-0 ,*spreading-0-frame*)
                   (:spreading-1 ,*spreading-1-frame*)
                   (:spreading-2 ,*spreading-2-frame*)
                   (:spreading-3 ,*spreading-3-frame*))))

(defparameter joint-position-designator
  (make-instance 'cram-designators:designator
    :description `((:joint-position 0.05 ))))

(defparameter stove-cooking-plate-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-cooking-plate-frame*))))

(defparameter drawer-left-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-table-drawer-left-frame*))))

(defparameter drawer-right-location-designator
  (make-instance 'cram-designators:designator
    :description `((:location-frame ,*stove-table-drawer-right-frame*))))
