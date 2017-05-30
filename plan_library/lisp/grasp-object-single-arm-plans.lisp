;;; Copyright (c) 2017, Boglarka Erdodi <erdoedib@cs.uni-bremen.de>
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

;;;===============================================;;;
;;;grasp object single arm (arm object-designator);;;
;;;===============================================;;;

(defun grasp-object-single-arm (arm object-designator
                                &optional new-pregrasping-frame new-grasping-frame)


  (let ((pregrasping-frame (if (eql new-pregrasping-frame NIL) :object-pre-grasping-frame new-pregrasping-frame))
        (grasping-frame (if (eql new-grasping-frame NIL) :object-grasping-frame new-grasping-frame)))
   ;;; sighted(grasping-frame)
  (sighted
   (tf-utilities::designator-property-as-location-designator object-designator pregrasping-frame))
  
   ;;; open gripper(arm distance effort)
  (singular-gripper-gripped
   arm
   0.070
   60.000)
  
  ;;; singular-arm-move-at-location(arm pregrasping-frame)
  (singular-arm-move-at-location
   arm
   (tf-utilities::designator-property-as-location-designator object-designator pregrasping-frame))

  ;;; singular-arm-move-at-location(arm grasping-frame)
  (singular-arm-move-at-location
   arm
   (tf-utilities::designator-property-as-location-designator object-designator grasping-frame))
  
  ;;; close-gripper(arm distance effort)
  (singular-gripper-gripped
   arm
   0.00
   60.000)

  ;;; the object is relative to robot's gripper.
  (roslisp:ros-info (grasp-object-single-arm) "Waiting for the object to be relative to the ~A gripper ..."  arm )
  (tf-utilities::update-object-location
   object-designator
   (cond ((eq arm :left) in-robot-left-gripper-designator)
         ((eq arm :right) in-robot-right-gripper-designator))
   (cl-transforms:transform-inv  (cram-designators:desig-prop-value object-designator grasping-frame)))))



