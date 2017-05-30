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

(defparameter *salt-cellar-initial-location-frame-name*
  "salt_cellar_initial_location_frame")

(defparameter *salt-cellar-frame-name*
  "salt_cellar_frame")

(defparameter *salt-cellar-over-frame-name*
  "salt_cellar_over_frame")

(defparameter *salt-cellar-pre-grasping-left-frame-name*
  "salt_cellar_pre_grasping_left_frame")

(defparameter *salt-cellar-pre-grasping-right-frame-name*
  "salt_cellar_pre_grasping_right_frame")

(defparameter *salt-cellar-grasping-left-frame-name*
  "salt_cellar_grasping_left_frame")

(defparameter *salt-cellar-grasping-right-frame-name*
  "salt_cellar_grasping_right_frame")

(defparameter *salting-left-first-frame-name*
  "salting_left_first_frame")

(defparameter *salting-right-first-frame-name*
  "salting_right_first_frame")

(defparameter *salting-right-second-frame-name*
  "salting_right_second_frame")

(defparameter *salting-right-third-frame-name*
  "salting_right_third_frame")

(defparameter *salt-cellar-front-pre-grasping-frame-name*
  "salt_cellar_front_pre_grasping_frame")

(defparameter *salt-cellar-front-grasping-frame-name*
  "salt_cellar_front_grasping_frame")

(defparameter *salt-cellar-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *salt-cellar-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector 0.08 0.40 -0.03)
    :rotation (cl-transforms:euler->quaternion
               ;; :ax (/ 3.14 3.0)
               ;; :ay (/ 3.14 2.0)
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *salt-cellar-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-initial-location-frame-name*
    :child-frame-id *salt-cellar-frame-name*
    :translation (cl-transforms:make-3d-vector 0.00 0.00 0.00)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *salt-cellar-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salt-cellar-over-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.25)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *salt-cellar-pre-grasping-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salt-cellar-pre-grasping-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.00 0.06 0.06)
    :rotation (cl-transforms:euler->quaternion
                :ax 0
                :ay 0
                :az (/ -3.14 2.0) )))

(defparameter *salt-cellar-pre-grasping-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salt-cellar-pre-grasping-right-frame-name*
    :translation (cl-transforms:make-3d-vector 0.00 -0.06 0.10)
    :rotation (cl-transforms:euler->quaternion
                :ax 0
                :ay 0
                :az (/ 3.14 2.0) )))

(defparameter *salt-cellar-grasping-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salt-cellar-grasping-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.06)
    :rotation (cl-transforms:euler->quaternion
                :ax 0
                :ay 0
                :az (/ -3.14 2.0) )))

(defparameter *salt-cellar-grasping-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salt-cellar-grasping-right-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:euler->quaternion
                :ax 0
                :ay 0
                :az (/ 3.14 2.0))))

(defparameter *salting-left-first-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salting-left-first-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.06)
    :rotation (cl-transforms:euler->quaternion
               ; :ax (/ -3.14 1.0)
               ; :ay 0
                :az (/ 3.14 1.0)
                )))

(defparameter *salting-right-first-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salting-right-first-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:euler->quaternion
              ; :ax (/ -3.14 1.0)
              ; :ay 0
               :az (/ 3.14 2.0)
                )))

(defparameter *salting-right-second-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salting-right-second-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:euler->quaternion
              ; :ax (/ -3.14 1.0)
              ; :ay 0
               :az (* 3.14 0.66)
               )))

(defparameter *salting-right-third-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salting-right-third-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:euler->quaternion
              ; :ax (/ -3.14 1.0)
              ; :ay 0
               :az (* 3.14 0.833)
               )))

(defparameter *salt-cellar-front-pre-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salt-cellar-front-pre-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.06 0.00 0.10)
    :rotation (cl-transforms:euler->quaternion
              ; :ax (/ 3.14 1)
              ; :ay  (/ 3.14 2)
              ; :az (/ 3.14 2)
               )))


(defparameter *salt-cellar-front-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *salt-cellar-frame-name*
    :child-frame-id *salt-cellar-front-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.10)
    :rotation (cl-transforms:euler->quaternion
              ; :ax (/ 3.14 1)
              ; :ay (/ 3.14 2)
              ; :az (/ 3.14 2)
               )))



(defun publish-salt-cellar-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *salt-cellar-initial-location-frame*
   *salt-cellar-frame*
   *salt-cellar-over-frame*
   *salt-cellar-pre-grasping-left-frame*
   *salt-cellar-pre-grasping-right-frame*
   *salt-cellar-grasping-left-frame*
   *salt-cellar-grasping-right-frame*
   *salting-left-first-frame*
   *salting-right-first-frame*
   *salting-right-second-frame*
   *salting-right-third-frame*
   *salt-cellar-front-pre-grasping-frame*
   *salt-cellar-front-grasping-frame*
   ))
