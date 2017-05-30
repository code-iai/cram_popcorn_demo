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

(in-package plan-library)

(defun move-arm-into-ready-position (arm)
  "This function is just auxiliary. It should be moved into some
  infrastructure."
  (cond
    ((eq arm :left)
     (progn
       (roslisp:wait-for-service "/execute_left_arm_cartesian_ik_trajectory")
       
       (roslisp:call-service
        "/execute_left_arm_cartesian_ik_trajectory"
        'two_hand_ik_trajectory_executor-srv:ExecuteLeftArmCartesianIKTrajectory
        :header (make-instance 'std_msgs-msg:header
                  :frame_id "base_link"
                  :stamp (roslisp:ros-time))
        :poses (vector
                (make-instance 'geometry_msgs-msg:pose
                  :position    (make-instance
                                   'geometry_msgs-msg:point
                                 :x 0.000 :y 0.700 :z 1.050)
                  :orientation (make-instance
                                   'geometry_msgs-msg:quaternion
                                 :w 1.0 :x 0.0 :y 0.0 :z 0.0))))))

    ((eql arm :right)
     (progn
       (roslisp:wait-for-service "/execute_right_arm_cartesian_ik_trajectory")
       
       (roslisp:call-service
        "/execute_right_arm_cartesian_ik_trajectory"
        'two_hand_ik_trajectory_executor-srv:ExecuteRightArmCartesianIKTrajectory
        :header (make-instance 'std_msgs-msg:header
                  :frame_id "base_link"
                  :stamp (roslisp:ros-time))
        :poses (vector
                (make-instance 'geometry_msgs-msg:pose
                  :position    (make-instance
                                   'geometry_msgs-msg:point
                                 :x 0.000 :y -0.700 :z 1.050)
                  :orientation (make-instance
                                   'geometry_msgs-msg:quaternion
                                 :w 1.0 :x 0.0 :y 0.0 :z 0.0))))))

    (t
     (roslisp:ros-warn
      'singular-arm-manipulation-ros-interface
      "There is no clear specification on which arm should execute the given trajectory !!!"))))

(defun move-both-arms-into-ready-position ()
  "This function is just auxiliary. It should be moved into some
  infrastructure."
  
  (roslisp:wait-for-service "/execute_both_arms_cartesian_ik_trajectory")
  
  (roslisp:call-service 
   "/execute_both_arms_cartesian_ik_trajectory" 
   'two_hand_ik_trajectory_executor-srv:ExecuteBothArmsCartesianIKTrajectory
   :header (make-instance 'std_msgs-msg:header 
             :frame_id "base_link"
             :stamp 0)
   :lh_poses (vector
              (make-instance 'geometry_msgs-msg:pose 
                :position    (make-instance
                                 'geometry_msgs-msg:point
                                 :x 0.000 :y 0.800 :z 1.050)
                :orientation (make-instance
                                 'geometry_msgs-msg:quaternion
                                 :w 1.0 :x 0.0 :y 0.0 :z 0.0)))
   :rh_poses (vector
              (make-instance 'geometry_msgs-msg:pose 
                :position    (make-instance
                                 'geometry_msgs-msg:point
                                 :x 0.000 :y -0.800 :z 1.050)
                :orientation (make-instance
                                 'geometry_msgs-msg:quaternion
                                 :w 1.0 :x 0.0 :y 0.0 :z 0.0)))
   :stop_at_pose (vector 1)))

;;
;; single arm
;;

(defun send-trajectory-to-cartesian-controller
    (arm trajectory)

  (assert arm)
  (assert trajectory)

  ;;TODO (lisca): Adapt the controller such that it checks the flags
  ;;to stop or not at a specific pose given in the trajectory. As in
  ;;both arm manipulation.
  (cond
    ((eq arm :left)
     (progn
       (roslisp:wait-for-service "/execute_left_arm_cartesian_ik_trajectory")
       
       (roslisp:call-service
        "/execute_left_arm_cartesian_ik_trajectory"
        'two_hand_ik_trajectory_executor-srv:ExecuteLeftArmCartesianIKTrajectory
        :header (make-instance 'std_msgs-msg:header
                  :frame_id "base_link"
                  :stamp (roslisp:ros-time))
        :poses trajectory)))

    ((eql arm :right)
     (progn
       (roslisp:wait-for-service "/execute_right_arm_cartesian_ik_trajectory")
       
       (roslisp:call-service
        "/execute_right_arm_cartesian_ik_trajectory"
        'two_hand_ik_trajectory_executor-srv:ExecuteRightArmCartesianIKTrajectory
        :header (make-instance 'std_msgs-msg:header
                  :frame_id "base_link"
                  :stamp (roslisp:ros-time))
        :poses trajectory)))

    (t
     (roslisp:ros-warn
      'singular-arm-manipulation-ros-interface
      "There is no clear specification on which arm should execute the
      given trajectory !!!"))))

;; TODO (lisca): Check for possible flaws in this macro!
;; USE gensym for `hand-trajectory' variable.
(defmacro execute-arm-trajectory
    (arm (&rest arm-trajectory-frame-names))
  `(let ((hand-trajectory
           (tf-utilities::destination-frame-names-list->base_link-trajectory-transforms-list
            ,arm
            ,@arm-trajectory-frame-names)))
     (send-trajectory-to-cartesian-controller
      ,arm hand-trajectory)))

;;
;; both arms
;;

;; TODO (lisca): Check for possible flaws in this macro!
(defun send-both-trajectories-to-cartesian-controller
    (left-hand-trajectory right-hand-trajectory)

  (assert
   (eq (length left-hand-trajectory)
       (length right-hand-trajectory)))
  
  (let ((trajectory-length
          (length left-hand-trajectory)))

    ;;TODO (lisca): Change the cartesian controller such that it
    ;; doesn't need this vector anymore
    ;; 0 To NOT stop at the pose from 
    ;; 1 To stop at the pose from index
    (let ((stop-at-flagged-idexes
            (make-array trajectory-length :initial-element 0)))

      (roslisp:wait-for-service "/execute_both_arms_cartesian_ik_trajectory")
      
      ;; Just call the cartesian controller service
      (roslisp:call-service
       "/execute_both_arms_cartesian_ik_trajectory" 
       'two_hand_ik_trajectory_executor-srv:ExecuteBothArmsCartesianIKTrajectory
       :header (make-instance 'std_msgs-msg:header 
                 :frame_id "base_link"
                 :stamp (roslisp:ros-time))
       :lh_poses left-hand-trajectory
       :rh_poses right-hand-trajectory
       :stop_at_pose stop-at-flagged-idexes))))

;; TODO (lisca): Check for possible flaws in this macro!
;; USE gensym for `left-hand-trajectory' and `right-hand-trajectory'
;; variables.
(defmacro execute-both-arms-trajectories
    ((&rest left-arm-trajectory-frame-names)
     (&rest right-arm-trajectory-frame-names))
  "For each arm take the frames trough which the gripper_tool_frame
   has to go and send them to the controller"
  `(let ((left-hand-trajectory
           (tf-utilities::destination-frame-names-list->base_link-trajectory-transforms-list
            :left
            ,@left-arm-trajectory-frame-names))
         (right-hand-trajectory
           (tf-utilities::destination-frame-names-list->base_link-trajectory-transforms-list
            :right
            ,@right-arm-trajectory-frame-names)))
     (send-both-trajectories-to-cartesian-controller
      left-hand-trajectory
      right-hand-trajectory)))
