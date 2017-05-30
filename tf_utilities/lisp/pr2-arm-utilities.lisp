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

(in-package tf-utilities)

(defparameter *wrist-left->gripper-left-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0.216 0.0 0.0)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defparameter *wrist-right->gripper-right-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0.18 0.0 0.0)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defun gripper_tool_frame-goal->wrist_frame-goal
  (arm gripper-tool-frame-goal)
  "Where should be the wrist_frame if the gripper_tool_frame is at
  `gripper-tool-transform'? Transforms are assumed relative to the
  base_link. Return the transform in base_link"

  ;; TODO (lisca): Check the type of gripper-tool-transform

  (cond
    ((eq arm :left)
     (cl-transforms:transform*
      gripper-tool-frame-goal
      (cl-transforms:transform-inv
       *wrist-left->gripper-left-transform*)))
    ((eq arm :right)
     (cl-transforms:transform*
      gripper-tool-frame-goal
      (cl-transforms:transform-inv
       *wrist-right->gripper-right-transform*)))))

(defun destination-frame-names-list->base_link-trajectory-transforms-list
    (arm &rest destination-frame-names-list)

  (mapcar
   #'(lambda (destination-frame-name)
       ;; Build a ROS message
       (cl-transform->geometry_msgs-pose
        ;; Ask in which pose the wrist of the right arm suppose to
        ;; be such that the end effector of the right gripper is in
        ;; the desired pose.
        (gripper_tool_frame-goal->wrist_frame-goal
         arm
         ;; Mare sure the given pose for the wrist are in the
         ;; robot base_link frame.
         (lookup-current-frame-relative-to-refference-frame
          destination-frame-name "base_link"))))
   destination-frame-names-list))
