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

(defvar *gripper-action-left* nil)
(defvar *gripper-action-right* nil)

(defun init-dual-griper-grasping-handlers ()
  (setf *gripper-action-left*
     (actionlib:make-action-client
      "/l_gripper_controller/gripper_action"
      "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *gripper-action-right*
        (actionlib:make-action-client
         "/r_gripper_controller/gripper_action"
         "pr2_controllers_msgs/Pr2GripperCommandAction")))

(roslisp-utilities:register-ros-init-function
 init-dual-griper-grasping-handlers)

;;TODO(lisca): Execute the gripper opening / closing in separate threads
(defun set-both-grippers-opening-amount (both-grippers-opening-designator)
  ;; Use two threads to simultaneously set the left and right grippers
  ;; opening amount
  (let ((left-gripper-opening-thread
          (sb-thread:make-thread
           #'(lambda ()
               (actionlib:send-goal-and-wait
                *gripper-action-left*
                (actionlib:make-action-goal
                    *gripper-action-left*
                  (position command)
                  (cram-designators:desig-prop-value both-grippers-opening-designator :left-griper-opening-amount)
                  (max_effort command)
                  (cram-designators:desig-prop-value both-grippers-opening-designator :left-griper-effort))
                :result-timeout 1.0))))
        (right-gripper-opening-thread
          (sb-thread:make-thread
           #'(lambda ()
               (actionlib:send-goal-and-wait
                *gripper-action-right*                
                (actionlib:make-action-goal
                    *gripper-action-right*
                  (position command)
                  (cram-designators:desig-prop-value both-grippers-opening-designator :right-griper-opening-amount)
                  (max_effort command)
                  (cram-designators:desig-prop-value both-grippers-opening-designator :right-griper-effort))
                :result-timeout 1.0)))))
    
    ;; Wait for both actions to finish
    (sb-thread:join-thread left-gripper-opening-thread)
    (sb-thread:join-thread right-gripper-opening-thread)))
