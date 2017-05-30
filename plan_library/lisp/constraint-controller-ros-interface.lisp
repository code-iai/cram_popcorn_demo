;;; Copyright (c) 2016, Gheorghe Lisca <lisca@cs.uni-bremen.de>
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

(in-package plan-library)

;;
;; constraints controller
;;

(defvar *giskard-action* nil)

(defun init-giskard-controller ()
  (setf *giskard-action*
     (actionlib:make-action-client
      "/controller_action_server/move"
      "giskard_msgs/WholeBodyAction")))

(roslisp-utilities:register-ros-init-function
 init-giskard-controller)

(defparameter *joint-motion-maximum-duration* 8.0)

(defun perform-joint-motion
    (&key (left-goal-joint-configuration-list nil)
          (right-goal-joint-configuration-list ))
  
  (roslisp:ros-info 'constraint-controller "start joint constrained motion ...")

  (cond ((and left-goal-joint-configuration-list right-goal-joint-configuration-list)
         
         (actionlib:send-goal-and-wait
          plan-library::*giskard-action*

          ;; goals for both grippers
          (actionlib:make-action-goal
              plan-library::*giskard-action*

            ;; 0=standard_controller
            (type command) 0

            ;; 2=joint motion
            (type left_ee command) 2
            ;; goal pose for the left end effector
            (goal_configuration left_ee command) (coerce left-goal-joint-configuration-list 'vector)
            ;;
            (convergence_thresholds left_ee command) (vector (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                   (semantics) "left_arm_0_error"
                                                                                   (value) 0.01)
                                                             (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                   (semantics) "left_arm_1_error"
                                                                                   (value) 0.01))
            ;; 2=joint motion
            (type right_ee command) 2
            ;; goal pose for the right end effector
            (goal_configuration right_ee command) (coerce right-goal-joint-configuration-list 'vector)
            ;;
            (convergence_thresholds right_ee command) (vector (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                   (semantics) "right_arm_0_error"
                                                                                   (value) 0.01)
                                                             (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                   (semantics) "right_arm_1_error"
                                                                                   (value) 0.01)))
          :result-timeout *joint-motion-maximum-duration*))

        (left-goal-joint-configuration-list
         ;; goal only for the left gripper
         (actionlib:send-goal-and-wait
          plan-library::*giskard-action*

          ;; goals for both grippers
          (actionlib:make-action-goal
              plan-library::*giskard-action*

            ;; 1=yaml_controller
            (type command) 0

            ;; 2=joint motion
            (type left_ee command) 2
            ;; goal pose for the left end effector
            (goal_configuration left_ee command) (coerce left-goal-joint-configuration-list 'vector)
            ;;
            (convergence_thresholds left_ee command) (vector (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                   (semantics) "left_arm_0_error"
                                                                                   (value) 0.01)
                                                             (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                   (semantics) "left_arm_1_error"
                                                                                   (value) 0.01)))
          
          :result-timeout *joint-motion-maximum-duration*))
        
        (right-goal-joint-configuration-list
         ;; goal only for the right gripper
         (actionlib:send-goal-and-wait
          plan-library::*giskard-action*

          ;; goals for the right arm
          (actionlib:make-action-goal
              plan-library::*giskard-action*

            ;; 1=yaml_controller
            (type command) 0

            ;; 2=joint motion
            (type right_ee command) 2
            ;; goal pose for the right arm
            (goal_configuration right_ee command) (coerce right-goal-joint-configuration-list 'vector)
            ;;
            (convergence_thresholds right_ee command) (vector (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                   (semantics) "right_arm_0_error"
                                                                                   (value) 0.01)
                                                             (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                   (semantics) "right_arm_1_error"
                                                                                   (value) 0.01)))
          
          :result-timeout *joint-motion-maximum-duration*)))

  (roslisp:ros-info 'constraint-controller "finished joint constrained motion ..."))

;;
;;
;;

(defparameter *cartesian-constrained-motion-maximum-duration* 10.0)

(defun perform-cartesian-constrained-motion
    (&key (left-gripper-goal-transform-stamped nil) (right-gripper-goal-transform-stamped nil)
       (left-translation-error 0.02) (right-translation-error 0.02)
       (left-rotation-error 0.05) (right-rotation-error 0.05))

  (declare (ignore right-rotation-error))
  (declare (ignore right-translation-error))
  
  (roslisp:ros-info (performing-cartesian-constrained-motion) "start cartesian constrained motion ...")

  (let (
        ;; ;; giskard gets goal poses in base_footprint frame
        ;; (giskard-left-gripper-goal-pose
        ;;   (if left-gripper-goal-transform-stamped
        ;;       (tf-utilities::transform-stamped->pose-stamped
        ;;        (tf-utilities::lookup-current-frame-relative-to-refference-frame
        ;;         (slot-value left-gripper-goal-transform-stamped 'cl-transforms-stamped:child-frame-id)
        ;;         "base_footprint"))
        ;;       nil))
        ;; (giskard-right-gripper-goal-pose
        ;;   (if right-gripper-goal-transform-stamped
        ;;       (tf-utilities::transform-stamped->pose-stamped
        ;;        (tf-utilities::lookup-current-frame-relative-to-refference-frame
        ;;         (slot-value right-gripper-goal-transform-stamped 'cl-transforms-stamped:child-frame-id)
        ;;         "base_footprint"))
        ;;       nil))

        ;; giskard gets poses in random frame
        ;; todo(lisca): why does giskard complain that the spoon_rounded_frame_projected is not in the
        ;;              same tf tree as base_footprint?
        (giskard-left-gripper-goal-pose
          (if left-gripper-goal-transform-stamped
              (progn
                (cl-transforms-stamped:restamp-transform-stamped left-gripper-goal-transform-stamped 0.0)
                (cl-transforms-stamped:transform-stamped->pose-stamped left-gripper-goal-transform-stamped))
              nil))
        (giskard-right-gripper-goal-pose
          (if right-gripper-goal-transform-stamped
              (progn
                (cl-transforms-stamped:restamp-transform-stamped right-gripper-goal-transform-stamped 0.0)
                (cl-transforms-stamped:transform-stamped->pose-stamped right-gripper-goal-transform-stamped))
              nil)))

    (cond ((and giskard-left-gripper-goal-pose giskard-right-gripper-goal-pose)

           (roslisp:ros-info (performing-cartesian-constrained-motion) "giskard action server called ...")
           
           (actionlib:send-goal-and-wait
            plan-library::*giskard-action*

            ;; goals for both grippers
            (actionlib:make-action-goal
                plan-library::*giskard-action*

              ;; 0=standard_controller
              (type command) 0

              ;; 1=cartesian motion
              (type left_ee command) 1
              ;; goal pose for the left end effector
              (goal_pose left_ee command) (cl-transforms-stamped:to-msg giskard-left-gripper-goal-pose)
              ;;
              (convergence_thresholds left_ee command) (vector (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                     (semantics) "l_trans_error"
                                                                                     (value) left-translation-error)
                                                               (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                     (semantics) "l_rot_error"
                                                                                     (value) left-rotation-error))
              ;; 1=cartesian motion
              (type right_ee command) 1
              ;; goal pose for the right end effector
              (goal_pose right_ee command) (cl-transforms-stamped:to-msg giskard-right-gripper-goal-pose)
              ;;
              (convergence_thresholds right_ee command) (vector (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                      (semantics) "r_trans_error"
                                                                                      (value) right-translation-error)
                                                                (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                      (semantics) "r_rot_error"
                                                                                      (value) right-rotation-error)))
            :result-timeout *cartesian-constrained-motion-maximum-duration*)

           (roslisp:ros-info (performing-cartesian-constrained-motion) "giskard action server returned ..."))

          (giskard-left-gripper-goal-pose

           (roslisp:ros-info (performing-cartesian-constrained-motion) "giskard action server called ...")
           
           ;; goal only for the left gripper
           (actionlib:send-goal-and-wait
            plan-library::*giskard-action*
            (actionlib:make-action-goal
                plan-library::*giskard-action*
              (type command) 0
              (type left_ee command) 1
              (goal_pose left_ee command) (cl-transforms-stamped:to-msg giskard-left-gripper-goal-pose)
              (convergence_thresholds left_ee command) (vector (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                     (semantics) "l_trans_error"
                                                                                     (value) left-translation-error)
                                                               (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                     (semantics) "l_rot_error"
                                                                                     (value) left-rotation-error)))
            :result-timeout *cartesian-constrained-motion-maximum-duration*)
           
           (roslisp:ros-info (performing-cartesian-constrained-motion) "giskard action server returned ..."))

          (giskard-right-gripper-goal-pose

           (roslisp:ros-info (performing-cartesian-constrained-motion) "giskard action server called ...")
           
           ;; goal only for the right gripper
           (actionlib:send-goal-and-wait
            plan-library::*giskard-action*
            (actionlib:make-action-goal
                plan-library::*giskard-action*
              (type command) 0
              (type right_ee command) 1
              (goal_pose right_ee command) (cl-transforms-stamped:to-msg giskard-right-gripper-goal-pose)
              (convergence_thresholds right_ee command) (vector (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                      (semantics) "r_trans_error"
                                                                                      (value) right-translation-error)
                                                                (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                                                      (semantics) "r_rot_error"
                                                                                      (value) right-rotation-error)))
            :result-timeout *cartesian-constrained-motion-maximum-duration*)
           
           (roslisp:ros-info (performing-cartesian-constrained-motion) "giskard action server returned ..."))))
  
  (roslisp:ros-info 'constraint-controller "finished cartesian constrained motion ..."))

