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

;;; ============================================= ;;;
;;; singular-arm-move-at-location (arm location)  ;;;
;;; ============================================= ;;;

(defun singular-arm-move-at-location (arm location-designator)

  ;; (let ((log-node-id (beliefstate:start-node "MOVE" nil)))
    
  ;;   (beliefstate:stop-node log-node-id :success T))

  (let ((gripper-tool-frame (cram-designators:desig-prop-value location-designator :location-frame)))

    (assert gripper-tool-frame)
      
    ;; inverse kinematics
    (execute-arm-trajectory
        arm ((cl-transforms-stamped:child-frame-id gripper-tool-frame)))

    ;; ;; constraint controller
    ;; (cond ((eq arm :left)
             
    ;;        (perform-cartesian-constrained-motion
    ;;         :left-gripper-goal-transform-stamped gripper-tool-frame))
            
    ;;       ((eq arm :right)
    ;;        (perform-cartesian-constrained-motion
    ;;         :right-gripper-goal-transform-stamped gripper-tool-frame))
            
    ;;       (t
    ;;        (roslisp:ros-warn (singular-arm-move-at-location) "please specifiy the arm which should move ...")))
    ))


;;; ============================================================================ ;;;
;;; singular-arm-reach-in ( arm reaching-in-designator )                         ;;;
;;; ============================================================================ ;;;

(defun singular-arm-reach-in (arm reaching-in-designator)

  (let ((log-node-id (beliefstate:start-node "SINGULAR-ARM-REACH-IN" nil)))

    (singular-arm-move-at-location
     arm (tf-utilities::designator-property-as-location-designator
          reaching-in-designator :pre-grasping-position))

    (singular-arm-move-at-location
     arm (tf-utilities::designator-property-as-location-designator
          reaching-in-designator :grasping-position))
    
    (beliefstate:stop-node log-node-id :success T)))


;;; ============================================================================ ;;;
;;; singular-arm-reach-out ( arm reaching-out-designator )                       ;;;
;;; ============================================================================ ;;;

(defun singular-arm-reach-out (arm reaching-out-designator)

  (let ((log-node-id (beliefstate:start-node "SINGULAR-ARM-REACH-OUT" nil)))    

    (singular-arm-move-at-location
     arm (tf-utilities::designator-property-as-location-designator
          reaching-out-designator :grasping-position))

    (singular-arm-move-at-location
     arm (tf-utilities::designator-property-as-location-designator
          reaching-out-designator :pre-grasping-position))
    
    (beliefstate:stop-node log-node-id :success T)))
