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


;;stove table frame names
(defparameter *stove-table-initial-location-frame-name*
  "stove_table_initial_location_frame")

(defparameter *stove-table-frame-name*
  "stove_table_frame")

(defparameter *stove-table-away-center-frame-name*
  "stove_table_away_center_frame")

(defparameter *stove-table-away-left-frame-name*
  "stove_table_away_left_frame")

(defparameter *stove-table-away-right-frame-name*
  "stove_table_away_right_frame")

(defparameter *stove-table-close-to-center-frame-name*
  "stove_table_close_to_center_frame")

(defparameter *stove-table-close-to-left-frame-name*
  "stove_table_close_to_left_frame")

(defparameter *stove-table-close-to-right-frame-name*
  "stove_table_close_to_right_frame")



(defparameter *stove-cooking-plate-frame-name*
  "stove_cooking_plate_frame")

(defparameter *stove-cooking-plate-left-frame-name*
  "stove_cooking_plate_left_frame")

(defparameter *stove-cooking-plate-right-frame-name*
  "stove_cooking_plate_right_frame")

(defparameter *stove-over-cooking-plate-left-frame-name*
  "stove_over_cooking_plate_left_frame")

(defparameter *stove-over-cooking-plate-right-frame-name*
  "stove_over_cooking_plate_right_frame")



(defparameter *stove-region-left-frame-name*
  "stove_region_left_frame")

(defparameter *stove-region-right-frame-name*
  "stove_region_right_frame")

(defparameter *stove-over-region-left-frame-name*
  "stove_over_region_left_frame")

(defparameter *stove-over-region-right-frame-name*
  "stove_over_region_right_frame")

(defparameter *stove-region-left-corner-frame-name*
  "stove_region_left_corner_frame")

(defparameter *stove-region-left-corner-2-frame-name*
  "stove_region_left_corner_2_frame")


(defparameter *stove-region-right-corner-frame-name*
  "stove_region_right_corner_frame")

(defparameter *stove-over-region-left-corner-frame-name*
  "stove_over_region_left_corner_frame")

(defparameter *stove-over-region-right-corner-frame-name*
  "stove_over_region_right_corner_frame")



;;stove knob frame names
(defparameter *stove-knob-pregrasping-on-frame-name*
  "stove_knob_pregrasping_on_frame")

(defparameter *stove-knob-pregrasping-off-frame-name*
  "stove_knob_pregrasping_off_frame")

(defparameter *stove-knob-grasping-on-frame-name*
  "stove_knob_grasping_on_frame")

(defparameter *stove-knob-grasping-off-frame-name*
  "stove_knob_grasping_off_frame")

(defparameter *stove-knob-on-frame-name*
   "stove_knob_on_frame")

(defparameter *stove-knob-off-frame-name*
   "stove_knob_off_frame")

(defparameter *stove-knob-initial-location-frame-name*
   "stove_knob_initial_location_frame")

(defparameter *stove-knob-on-object-frame-name*
   "stove_knob_on_object_frame")

(defparameter *stove-knob-off-object-frame-name*
   "stove_knob_off_object_frame")


;;stove drawers opening position names
(defparameter *stove-open-drawer-left-frame-name*
  "stove_open_drawer_left_frame")

(defparameter *stove-open-drawer-right-frame-name*
  "stove_open_drawer_right_frame")



(defparameter *stove-table-initial-location-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id "map"
    :child-frame-id *stove-table-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector 0.68 -0.05 0.05)
    :rotation (cl-transforms:make-identity-rotation)))
  
(defparameter *stove-table-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id  *stove-table-initial-location-frame-name*
    :child-frame-id *stove-table-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-table-away-center-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-table-away-center-frame-name*
    :translation (cl-transforms:make-3d-vector -1.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-table-away-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-table-away-left-frame-name*
    :translation (cl-transforms:make-3d-vector -1.0 0.5 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-table-away-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-table-away-right-frame-name*
    :translation (cl-transforms:make-3d-vector -1.0 -0.5 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-table-close-to-center-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-table-close-to-center-frame-name*
    :translation (cl-transforms:make-3d-vector -0.8 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-table-close-to-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-table-close-to-left-frame-name*
    :translation (cl-transforms:make-3d-vector -0.8 0.5 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-table-close-to-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-table-close-to-right-frame-name*
    :translation (cl-transforms:make-3d-vector -0.8 -0.5 0.0)
    :rotation (cl-transforms:make-identity-rotation)))



(defparameter *stove-cooking-plate-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-cooking-plate-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.69)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-cooking-plate-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-cooking-plate-left-frame-name*
    :translation (cl-transforms:make-3d-vector -0.15 0.16 -0.01)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-cooking-plate-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-cooking-plate-right-frame-name*
    :translation (cl-transforms:make-3d-vector  -0.15 -0.04 -0.01)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-over-cooking-plate-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-left-frame-name*
    :child-frame-id *stove-over-cooking-plate-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.30)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-over-cooking-plate-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-right-frame-name*
    :child-frame-id *stove-over-cooking-plate-right-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.30)
    :rotation (cl-transforms:make-identity-rotation)))


(defparameter *stove-region-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-region-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.55 0.0)
    :rotation (cl-transforms:make-identity-rotation)))


(defparameter *stove-region-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-region-right-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -0.55 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-over-region-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-over-region-left-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0465 0.6075 0.3608)
     :rotation (cl-transforms:normalize
               (cl-transforms:make-quaternion -0.1902 0.1989 0.0102 -0.9613))))

(defparameter *stove-over-region-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-over-region-right-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 -0.55 0.15)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-region-left-corner-frame* 
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-region-left-corner-frame-name*
    :translation (cl-transforms:make-3d-vector -0.18 0.60 0.00)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-region-left-corner-2-frame* 
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-region-left-corner-2-frame-name*
    :translation (cl-transforms:make-3d-vector  0.08 0.40 -0.03)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-region-right-corner-frame* 
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-region-right-corner-frame-name*
    :translation (cl-transforms:make-3d-vector  -0.22 -0.47 0.00)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-over-region-left-corner-frame* 
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-over-region-left-corner-frame-name*
    :translation (cl-transforms:make-3d-vector  -0.15 0.55 0.15)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-over-region-right-corner-frame* 
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-over-region-right-corner-frame-name*
    :translation (cl-transforms:make-3d-vector  -0.22 -0.47 0.15)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-knob-initial-location-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-cooking-plate-frame-name*
    :child-frame-id *stove-knob-initial-location-frame-name*
    :translation (cl-transforms:make-3d-vector -0.22 -0.22 0.0)
    :rotation (cl-transforms:make-identity-rotation)))


(defparameter *stove-knob-on-object-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id  *stove-knob-initial-location-frame-name*
    :child-frame-id *stove-knob-on-object-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))

(defparameter *stove-knob-off-object-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id  *stove-knob-initial-location-frame-name*
    :child-frame-id *stove-knob-off-object-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:make-identity-rotation)))


(defparameter *stove-knob-pregrasping-on-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-knob-initial-location-frame-name*
    :child-frame-id *stove-knob-pregrasping-on-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.05)
    :rotation (cl-transforms:euler->quaternion
             ; :ax (/  3.14 2.0)
               :ay (/  3.14 2.0)
               :az (/  3.14 2.0))))

(defparameter *stove-knob-pregrasping-off-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-knob-initial-location-frame-name*
    :child-frame-id *stove-knob-pregrasping-off-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.05)
    :rotation (cl-transforms:euler->quaternion
             ; :ax (/  3.14 2.0)
               :ay (/  3.14 2.0)
               :az (/  -3.14 2.0))))


(defparameter *stove-knob-grasping-on-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-knob-initial-location-frame-name*
    :child-frame-id *stove-knob-grasping-on-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:euler->quaternion
             ; :ax (/  3.14 2.0)
               :ay (/  3.14 2.0)
               :az (/  3.14 2.0))))

(defparameter *stove-knob-grasping-off-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-knob-initial-location-frame-name*
    :child-frame-id *stove-knob-grasping-off-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:euler->quaternion
             ; :ax (/  3.14 2.0)
               :ay (/  3.14 2.0)
               :az (/  -3.14 2.0))))


(defparameter *stove-knob-on-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-knob-initial-location-frame-name*
    :child-frame-id *stove-knob-on-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:euler->quaternion
             ; :ax (/  3.14 2.0)
               :ay (/  3.14 2.0)
               :az (/ -3.14 2.0))))

(defparameter *stove-knob-off-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-knob-initial-location-frame-name*
    :child-frame-id *stove-knob-off-frame-name*
    :translation (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    :rotation (cl-transforms:euler->quaternion
             ; :ax (/ 3.14 2.0)
               :ay (/ 3.14 2.0)
               :az (/ 3.14 2.0 ))))

(defparameter *stove-open-drawer-left-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-open-drawer-left-frame-name*
    :translation (cl-transforms:make-3d-vector -0.43 0.52 0.47)
    :rotation  (cl-transforms:make-identity-rotation)))

(defparameter *stove-open-drawer-right-frame*
  (make-instance
      'cl-transforms-stamped:transform-stamped
    :stamp 0.0
    :frame-id *stove-table-frame-name*
    :child-frame-id *stove-open-drawer-right-frame-name*
    :translation (cl-transforms:make-3d-vector -0.48 -0.52 0.47)
    :rotation  (cl-transforms:make-identity-rotation)))


(defun publish-stove-table-frames ()
  (cl-transforms-stamped:start-publishing-transforms-static-globaly

   *stove-table-initial-location-frame*
   *stove-table-frame*
   *stove-table-away-center-frame*
   *stove-table-away-left-frame*
   *stove-table-away-right-frame*
   *stove-table-close-to-center-frame*
   *stove-table-close-to-left-frame*
   *stove-table-close-to-right-frame*
   
   *stove-cooking-plate-frame*
   *stove-cooking-plate-left-frame*
   *stove-cooking-plate-right-frame*
   *stove-over-cooking-plate-left-frame*
   *stove-over-cooking-plate-right-frame*

   *stove-region-left-frame*
   *stove-region-right-frame*
   *stove-over-region-left-frame*
   *stove-over-region-right-frame*
   *stove-region-left-corner-frame*
   *stove-region-left-corner-2-frame*
   *stove-region-right-corner-frame*
   *stove-over-region-left-corner-frame*
   *stove-over-region-right-corner-frame*

   *stove-knob-pregrasping-on-frame*
   *stove-knob-pregrasping-off-frame*
   *stove-knob-grasping-on-frame*
   *stove-knob-grasping-off-frame*
   *stove-knob-on-frame*
   *stove-knob-off-frame*
   *stove-knob-initial-location-frame*
   *stove-knob-on-object-frame*
   *stove-knob-off-object-frame*
   

   *stove-open-drawer-left-frame*
   *stove-open-drawer-right-frame*))
