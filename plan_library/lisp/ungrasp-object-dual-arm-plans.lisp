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

;;;================================================================================================;;;
;;;ungrasp object dual arm ( object-designator location-designator both-grippers-open-designator)  ;;;
;;;================================================================================================;;;

(defun ungrasp-object-dual-arm (object-designator location-designator both-grippers-open-designator)

   ;;; sighted(grasping-frame)
   (roslisp:ros-info (ungrasp-object-dual-arm) "Sighting to object grasping left frame..." )
   (sighted
   (tf-utilities::designator-property-as-location-designator object-designator :object-grasping-left-frame))
  
  ;;; open dual-gripper (both-grippers-opening-designator)
  (roslisp:ros-info (ungrasp-object-dual-arm) "Opening dual-gripper ..." )
  (dual-gripper-gripped
   both-grippers-open-designator)

   ;;; update container's location relative to the destination location.
  (roslisp:ros-info (ungrasp-object-dual-arm) "Update object's location relative to the destination location..." )
  (tf-utilities::update-object-location
   object-designator
   location-designator
   (cl-transforms:make-identity-transform))
  (roslisp:ros-info (ungrasp-object-dual-arm) "The object is relative to the destination location ..." )

   ;;; move both grippers in the pregrasping frame
  (roslisp:ros-info (ungrasp-object-dual-arm) "Moving both grippers in the pregrasping frame ..." )
  (dual-arm-move-at-locations
   (tf-utilities::designator-property-as-location-designator object-designator :object-pre-grasping-left-frame)
   (tf-utilities::designator-property-as-location-designator object-designator :object-pre-grasping-right-frame))

  ;;;  move both arms in the ready position
  (roslisp:ros-info (ungrasp-object-dual-arm) "Moving both arms in the ready position..." )
  (dual-arm-move-at-locations
     designator-library::pr2-ready-arm-left-designator
     designator-library::pr2-ready-arm-right-designator))
 
