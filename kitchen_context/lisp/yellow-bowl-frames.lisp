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

(defparameter *yellow-bowl-initial-location-frame-name*
  "yellow_bowl_initial_location_frame")


(defparameter *yellow-bowl-frame-name*
  "yellow_bowl_frame")


(defparameter *yellow-bowl-over-frame-name*
  "yellow_bowl_over_frame")


;; (defparameter *yellow-bowl-scooping-0-frame-name*
;;   "yellow_bowl_scooping_0_frame")

;; (defparameter *yellow-bowl-scooping-1-frame-name*
;;   "yellow_bowl_scooping_1_frame")

;; (defparameter *yellow-bowl-scooping-2-frame-name*
;;   "yellow_bowl_scooping_2_frame")


(defparameter *yellow-bowl-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *kitchen-island-frame-name*
    :child-frame-id *yellow-bowl-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector 0.1 0.90 0.86)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay 3.14
               ;; :az (/ -3.14 2.0)
               )))

(defparameter *yellow-bowl-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *yellow-bowl-initial-location-frame-name*
    :child-frame-id *yellow-bowl-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *yellow-bowl-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *yellow-bowl-frame-name*
    :child-frame-id *yellow-bowl-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.038 0.024 0.20)
    :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.387 -0.147 0.572 0.708))))

;; (defparameter *yellow-bowl-scooping-0-frame*
;;   (make-instance
;;       'cl-transforms-stamped:transform-stamped
;;     :stamp 0.0
;;     :frame-id *yellow-bowl-frame-name*
;;     :child-frame-id *yellow-bowl-scooping-0-frame-name*
;;     :translation (cl-transforms:make-3d-vector 0.0 0.07 0.035)
;;     :rotation (cl-transforms:euler->quaternion
;;                :ax (/ -3.14 5.0)
;;                :ay (/ -3.14 6.0)
;;                ;; :az (/ 3.14 6.0)
;;                )))

;; (defparameter *yellow-bowl-scooping-1-frame*
;;   (make-instance
;;       'cl-transforms-stamped:transform-stamped
;;     :stamp 0.0
;;     :frame-id *yellow-bowl-frame-name*
;;     :child-frame-id *yellow-bowl-scooping-1-frame-name*
;;     :translation (cl-transforms:make-3d-vector 0.0 0.0 0.03)
;;     :rotation (cl-transforms:euler->quaternion
;;                :ax (/ -3.14 3.5)
;;              ;;:ay (/ -3.14 6.0)
;;                ;;:az (/ -3.14 6.0)
;;                )))

;; (defparameter *yellow-bowl-scooping-2-frame*
;;   (make-instance
;;       'cl-transforms-stamped:transform-stamped
;;     :stamp 0.0
;;     :frame-id *yellow-bowl-frame-name*
;;     :child-frame-id *yellow-bowl-scooping-2-frame-name*
;;     :translation (cl-transforms:make-3d-vector -0.05  0.0 0.25)
;;     :rotation (cl-transforms:euler->quaternion
;;                :ax (/ -3.14 2.0)
;;                ;;   :ay (/ -3.14 6.0)
;;                ;; :az (/ 3.14 2.0)
;;                )))

(defun publish-yellow-bowl-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *yellow-bowl-initial-location-frame*

   *yellow-bowl-frame*
   *yellow-bowl-over-frame*
   
   ;; *yellow-bowl-scooping-0-frame*
   ;; *yellow-bowl-scooping-1-frame*
   ;; *yellow-bowl-scooping-2-frame*
   ))
