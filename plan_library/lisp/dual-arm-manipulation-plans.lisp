;;; Copyright (c) 2013-16, Gheorghe Lisca <lisca@cs.uni-bremen.de>
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

;;; ==================================================================== ;;;
;;; dual-arm-move-at-locations (left-hand-location right-hand-location) ;;;
;;; ==================================================================== ;;;

(defun dual-arm-move-at-locations (left-gripper-location-designator right-gripper-location-designator)

  (let ((log-node-id (beliefstate:start-node "DUAL-ARM-MOVE-AT-LOCATIONS" nil)))

    (roslisp:ros-info 'dual-arm-move-at-locations "start simultaneoulsy moving both arms at specified locations ...")
    
    (let ((left-gripper-tool-frame
            (cram-designators:desig-prop-value left-gripper-location-designator :location-frame))
          (right-gripper-tool-frame
            (cram-designators:desig-prop-value right-gripper-location-designator :location-frame)))

      (assert left-gripper-tool-frame)
      (assert right-gripper-tool-frame)

      ;; inverse kinematics
      (execute-both-arms-trajectories
          ((cl-transforms-stamped:child-frame-id left-gripper-tool-frame))
          ((cl-transforms-stamped:child-frame-id right-gripper-tool-frame)))

      ;; (perform-cartesian-constrained-motion
      ;;  :left-gripper-goal-transform-stamped left-gripper-goal-frame
      ;;  :right-gripper-goal-transform-stamped right-gripper-goal-frame)
      )
    
    (roslisp:ros-info 'dual-arm-move-at-locations "finished simultaneoulsy moving both arms at specified locations ...")
    
    (beliefstate:stop-node log-node-id :success T)))

;;; ============================================ ;;;
;;; dual arm grasped ungrasped ( action object ) ;;;
;;; ============================================ ;;;

(defun dual-arm-grasped (action object-designator)

  (declare (ignore action))
  (declare (ignore object-designator))

  (roslisp:ros-info (dual-arm-grasped-ungrasped) "this plan is not implemented!")

  (let ((log-node-id (beliefstate:start-node "DUAL-ARM-REACH-UNREACH" nil)))    

    ;; (let ((dual-arm-grasping-ungrasping-designator
    ;;         (tf-utilities::create-dual-arm-grasping-ungrasping-designator
    ;;          action
    ;;          object-designator)))

    ;;   (let ((manipulation-type
    ;;           (cram-designators:desig-prop-value dual-arm-grasping-ungrasping-designator :to))
    
    ;;         (left-pre-grasping-frame
    ;;           (cram-designators:desig-prop-value dual-arm-grasping-ungrasping-designator :object-left-pre-grasping-frame))
    ;;         (left-grasping-frame
    ;;           (cram-designators:desig-prop-value dual-arm-grasping-ungrasping-designator :object-left-grasping-frame))
    ;;         (right-pre-grasping-frame
    ;;           (cram-designators:desig-prop-value dual-arm-grasping-ungrasping-designator :object-right-pre-grasping-frame))
    ;;         (right-grasping-frame
    ;;           (cram-designators:desig-prop-value dual-arm-grasping-ungrasping-designator :object-right-grasping-frame)))

    ;;     (assert manipulation-type)

    ;;     (cond ((eq manipulation-type
    ;;                :dual-arm-grasp)

    ;;            (assert left-pre-grasping-frame)
    ;;            (assert left-grasping-frame)
    ;;            (assert right-pre-grasping-frame)
    ;;            (assert right-grasping-frame)

    ;;            (beliefstate:add-designator-to-node
    ;;             dual-arm-grasping-ungrasping-designator log-node-id :annotation "dual-arm-reach")
    
    ;;            (execute-both-arms-trajectories
    ;;                (left-pre-grasping-frame
    ;;                 left-grasping-frame)
    ;;                (right-pre-grasping-frame
    ;;                 right-grasping-frame)))

    ;;           ((eq manipulation-type
    ;;                :dual-arm-ungrasp)

    ;;            (assert left-pre-grasping-frame)
    ;;            (assert left-grasping-frame)
    ;;            (assert right-pre-grasping-frame)
    ;;            (assert right-grasping-frame)

    ;;            (beliefstate:add-designator-to-node
    ;;             dual-arm-grasping-ungrasping-designator log-node-id :annotation "dual-arm-unreach")

    ;;            (execute-both-arms-trajectories
    ;;                (left-grasping-frame
    ;;                 left-pre-grasping-frame)
    ;;                (right-grasping-frame
    ;;                 right-pre-grasping-frame)))

    ;;           ((eq manipulation-type
    ;;                :dual-arm-move-at-location)

    ;;            (assert left-grasping-frame)
    ;;            (assert right-grasping-frame)

    ;;            (execute-both-arms-trajectories
    ;;                (left-grasping-frame)
    ;;                (right-grasping-frame)))

    ;;           ;; TODO (lisca): The previous cases are particular cases
    ;;           ;; of this one! Please get rid of them and implement this one!
    ;;           ((eq manipulation-type
    ;;                :execute-dual-arm-trajectories)
    ;;            (roslisp:ros-warn
    ;;             'dual-arm-manipulation-plans
    ;;             "execute-dual-arm-trajectories is NOT implemented yet!!!"))

    ;;           (t
    ;;            (roslisp:ros-warn
    ;;             'dual-arm-manipulation-plans
    ;;             "There is no handler able to execute ~a " manipulation-type)))))

    (beliefstate:stop-node log-node-id :success T)))

