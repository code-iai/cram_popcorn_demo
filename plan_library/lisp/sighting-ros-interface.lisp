;;; Copyright (c) 2013, Gheorghe Lisca <lisca@cs.uni-bremen.de>
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

(in-package :plan-library)

(defvar *move-hdead-action-client* nil)

(defun initialize-move-head-interface ()
  (setf *move-hdead-action-client*
        (actionlib:make-action-client
         "/head_traj_controller/point_head_action"
         "pr2_controllers_msgs/PointHeadAction")))

(roslisp-utilities:register-ros-init-function
  initialize-move-head-interface)

(defun send-sight-location-to-head-controller
    (x y z
     &key (reference-frame "base_link"))
  "TO NOT BE USED FOR LOCATION TRACKING !!!"

    (actionlib:send-goal-and-wait
     *move-hdead-action-client*
     (actionlib:make-action-goal
         *move-hdead-action-client*

       ;; NOTE: We didn't launch kinect in Gazebo. Therefore its
       ;;       frames are missing.
       (pointing_frame) "double_stereo_link"

       (x point target) x
       (y point target) y
       (z point target) z

       ;; in the reference-frame
       (frame_id header target) reference-frame

       ;; not faster than
       (max_velocity) 0.4 ;; Rad/s

       ;; Don't expose which axis of the pointing_frame to go trough
       ;; the given point (relative to the reference-frame)
       (x pointing_axis) 1.0
       (y pointing_axis) 0.0
       (z pointing_axis) 0.0)

     :result-timeout 20.0))

(defun sight-at-location (location)

  (assert location)

  (let ((frame-name-to-sight-at
          (cl-transforms-stamped:child-frame-id
           (cram-designators:desig-prop-value location :location-frame))))

    (assert frame-name-to-sight-at)

    (let ((frame-to-sight-at-in-base_link
            (tf-utilities::lookup-current-frame-relative-to-refference-frame
             frame-name-to-sight-at "base_link")))

      (assert frame-to-sight-at-in-base_link)

      (let ((location-3d-vector
              (cl-transforms:translation
               frame-to-sight-at-in-base_link)))

        (assert location-3d-vector)

        (send-sight-location-to-head-controller
         (cl-transforms:x location-3d-vector)
         (cl-transforms:y location-3d-vector)
         (- (cl-transforms:z location-3d-vector) 0.8))))))
