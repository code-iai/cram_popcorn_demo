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

(defvar *navigate-base-action-client* nil)

(defun initialize-navigate-base-action-client ()
  (setf *navigate-base-action-client*
        (actionlib:make-action-client
         "/nav_pcontroller/move_base"
         "move_base_msgs/MoveBaseAction")))

(roslisp-utilities:register-ros-init-function
 initialize-navigate-base-action-client)

(defun navigate-base-into-location-frame (location-frame-name)
  (let ((location-transform
          (tf-utilities::lookup-current-frame-relative-to-refference-frame
           location-frame-name "map")))
    (with-slots ((location-translation cl-transforms:translation)
                 (location-orientation cl-transforms:rotation)) location-transform
      (with-slots ((location-x cl-transforms:x)
                   (location-y cl-transforms:y)
                   (location-z cl-transforms:z)) location-translation
        (with-slots ((orientation-x cl-transforms:x)
                     (orientation-y cl-transforms:y)
                     (orientation-z cl-transforms:z)
                     (orientation-w cl-transforms:w)) location-orientation
          (actionlib:send-goal-and-wait
           *navigate-base-action-client*
           (actionlib:make-action-goal
               *navigate-base-action-client*
             (frame_id header target_pose) "map"
             (x position pose target_pose) location-x
             (y position pose target_pose) location-y
             (z position pose target_pose) location-z
             (x orientation pose target_pose) orientation-x
             (y orientation pose target_pose) orientation-y
             (z orientation pose target_pose) orientation-z
             (w orientation pose target_pose) orientation-w)))))))
