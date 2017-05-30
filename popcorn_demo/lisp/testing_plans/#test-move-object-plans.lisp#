
;;; Copyright (c) 2017, Mihaela Popescu <popescu@uni-bremen.de>
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


(in-package :popcorn-demo)


;;========= test to move object with one arm ===========
(defun execute-test-to-move-object-single-arm  ()

  (roslisp-utilities:startup-ros)
  (roslisp:ros-info 'popcorn-demo "Ros-node started.")

  (popcorn-demo::initialize-popcorn-demo)
  (roslisp:ros-info 'popcorn-demo "Popcorn-demo initialized.")
  (sleep 1)

  (plan-library::navigate-at-location
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-center))
  (roslisp:ros-info 'popcorn-demo "Navigation executed.")

  (plan-library::singular-arm-move-at-location
   :left
   (tf-utilities::designator-property-as-location-designator  kitchen-context::stove-table-drawer-left-designator :object-grasping-frame))
  (roslisp:ros-info 'popcorn-demo "Moved arm to grasp object.")

  
  (plan-library::single-arm-move-object-at-location
   :left
   kitchen-context::stove-table-drawer-left-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :open-left-drawer))
  (roslisp:ros-info 'popcorn-demo "Move object at location done.")
  
  (roslisp:ros-info 'popcorn-demo "All tests finished."))



;;========= test to move object with both arms ===========

(defun execute-test-to-move-object-double-arms  ()

  (roslisp-utilities:startup-ros)
  (roslisp:ros-info 'popcorn-demo "Ros-node started.")

  (popcorn-demo::initialize-popcorn-demo)
  (roslisp:ros-info 'popcorn-demo "Popcorn-demo initialized.")
  (sleep 1)

  (plan-library::navigate-at-location
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-right))
  (roslisp:ros-info 'popcorn-demo "Navigation executed.")

  (plan-library::dual-arm-move-at-locations
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :object-grasping-left-frame)
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :object-grasping-right-frame))
  (roslisp:ros-info 'popcorn-demo "Moved arms to grasp object.")

  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :over-region-right))
  (roslisp:ros-info 'popcorn-demo "Move object at location done.")
  
  (roslisp:ros-info 'popcorn-demo "All tests finished."))
