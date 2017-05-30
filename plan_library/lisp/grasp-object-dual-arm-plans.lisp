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

;;;========================================================================;;;
;;;grasp object dual arm (object-designator both-grippers-close-designator);;;
;;;========================================================================;;;


(defun grasp-object-dual-arm (object-designator both-grippers-close-designator)

  ;;; sighted(grasping-frame)
   (sighted
   (tf-utilities::designator-property-as-location-designator object-designator :object-grasping-left-frame))
  
  ;;; dual-arm-move-at-locations(pre-grasping-left  pre-grasping-right)
  (dual-arm-move-at-locations
   (tf-utilities::designator-property-as-location-designator object-designator :object-pre-grasping-left-frame)
   (tf-utilities::designator-property-as-location-designator object-designator :object-pre-grasping-right-frame))

  ;;; dual-arm-move-at-location(grasping-left grasping-right)
  (dual-arm-move-at-locations
   (tf-utilities::designator-property-as-location-designator object-designator :object-grasping-left-frame)
   (tf-utilities::designator-property-as-location-designator object-designator :object-grasping-right-frame))

  ;;; close-grippers(both-grippers-opening-designator)
  (dual-gripper-gripped
   both-grippers-close-designator)

  ;;; the object is relative to robot's left gripper.
  (tf-utilities::update-object-location
   object-designator
   in-robot-left-gripper-designator
   (cl-transforms:transform-inv (cram-designators:desig-prop-value object-designator :object-grasping-left-frame)))

  (roslisp:ros-info ( grasp-object-dual-arm) "the object is relative to the left gripper ..." ))
