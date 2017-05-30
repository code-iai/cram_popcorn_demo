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

(defparameter *stove-table-drawer-right-initial-location-frame-name*
  "stove_table_drawer_right_initial_location_frame")

(defparameter *stove-table-drawer-right-frame-name*
  "stove_table_drawer_right_frame")

(defparameter *stove-table-drawer-right-over-frame-name*
  "stove_table_drawer_right_over_frame")

(defparameter *stove-table-drawer-right-pre-grasping-frame-name*
  "stove_table_drawer_right_pre_grasping_frame")

(defparameter *stove-table-drawer-right-grasping-frame-name*
  "stove_table_drawer_right_grasping_frame") 


(defparameter *stove-table-drawer-right-initial-location-frame*  
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name* 
    :child-frame-id *stove-table-drawer-right-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector -0.05 -0.52 0.47)
    :rotation (cl-transforms:euler->quaternion 
               ;; :ax (/ 3.14 1.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *stove-table-drawer-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-drawer-right-initial-location-frame-name*
    :child-frame-id *stove-table-drawer-right-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:euler->quaternion 
               ;; :ax (/ 3.14 2.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *stove-table-drawer-right-over-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-drawer-right-frame-name*
    :child-frame-id *stove-table-drawer-right-over-frame-name*
    :translation (cl-transforms:make-3d-vector -0.1 -0.1 0.5)
    :rotation (cl-transforms:euler->quaternion 
               ;; :ax (/ 3.14 2.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *stove-table-drawer-right-pre-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-drawer-right-frame-name*
    :child-frame-id *stove-table-drawer-right-pre-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.350 0.0 0.07)
    :rotation (cl-transforms:euler->quaternion 
                :ax (/ 3.14 2.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))

(defparameter *stove-table-drawer-right-grasping-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-drawer-right-frame-name*
    :child-frame-id *stove-table-drawer-right-grasping-frame-name*
    :translation (cl-transforms:make-3d-vector -0.290 0.0 0.07)
    :rotation (cl-transforms:euler->quaternion 
                :ax (/ 3.14 2.0)
               ;; :ay 3.14
               ;; :az (/ 3.14 2.0)
               )))


(defun publish-stove-table-drawer-right-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *stove-table-drawer-right-initial-location-frame*
   *stove-table-drawer-right-frame*
   *stove-table-drawer-right-over-frame*
   *stove-table-drawer-right-pre-grasping-frame*
   *stove-table-drawer-right-grasping-frame*))
