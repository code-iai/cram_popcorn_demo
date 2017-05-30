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

(defparameter *boxy-left-gripper-publisher* nil)
(defparameter *boxy-right-gripper-publisher* nil)

(defun init-singular-griper-gripping-handlers ()
  (setf *gripper-action-left*
     (actionlib:make-action-client
      "/l_gripper_controller/gripper_action"
      "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *gripper-action-right*
        (actionlib:make-action-client
         "/r_gripper_controller/gripper_action"
         "pr2_controllers_msgs/Pr2GripperCommandAction"))

  ;; ;; advertise the topics!
  ;; (setf
  ;;  *boxy-left-gripper-publisher*
  ;;  (roslisp:advertise
  ;;   "/left_arm_gripper/goal_position" "iai_wsg_50_msgs/PositionCmd"
  ;;   ;; :latch "/left_arm_gripper/goal_position"
  ;;   ))

  ;; (setf
  ;;  *boxy-right-gripper-publisher*
  ;;  (roslisp:advertise
  ;;   "/right_arm_gripper/goal_position" "iai_wsg_50_msgs/PositionCmd"
  ;;   ;; :latch "/right_arm_gripper/goal_position"
  ;;   ))
  )

(roslisp-utilities:register-ros-init-function
 init-singular-griper-gripping-handlers)

;;TODO(lisca): Execute the gripper opening / closing
(defun set-gripper-opening-amount (gripper opening effort)
  (multiple-value-bind (result)
      
      (cond
        ((eq gripper :left)
         (actionlib:send-goal-and-wait
          *gripper-action-left*
          (actionlib:make-action-goal
              *gripper-action-left*

            (position command) opening

            (max_effort command) effort)
          :result-timeout 10.0))

        ((eq gripper :right)
         (actionlib:send-goal-and-wait
          *gripper-action-right*
          (actionlib:make-action-goal
              *gripper-action-right*

            (position command) opening

            (max_effort command) effort)
          :result-timeout 10.0))

        (t
         (roslisp:ros-warn
          'singular-gripper-gripping-plans
          "There is no clear specification on which gripper should be used")))
    result)

  ;; todo(lisca): remove this hack!
  (dolist (element (list 1 2 3))
    (declare (ignore element))
    
    ;; (cond ((eq gripper :left)
    ;;        ;; maybe one of boxy's left gripper is down
           
    ;;        (roslisp::publish
    ;;         *boxy-left-gripper-publisher*
    ;;         (roslisp::make-message "iai_wsg_50_msgs/PositionCmd" (pos) opening (speed) 30.0 (force) effort)))

    ;;       ((eq gripper :right)
    ;;        ;; maybe one of boxy's right gripper is down
    ;;        *boxy-right-gripper-publisher*
    ;;        (roslisp::make-message "iai_wsg_50_msgs/PositionCmd" (pos) opening (speed) 30.0 (force) effort)))
    ))
