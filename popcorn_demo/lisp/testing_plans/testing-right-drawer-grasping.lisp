;;; Copyright (c) 2017, Boglarka Erdodi <erdoedib@uni-bremen.de>
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

(in-package :popcorn-demo)  


;;; define the navigation to the stove table

(defun navigate-to-stove (location-designator)
  (plan-library::navigate-at-location location-designator))


;;; execute grasping the right drawer

(defun grasping-right-drawer ()

  (roslisp-utilities:startup-ros)
  (roslisp:ros-info 'popcorn-demo "Ros-node started.")

  (popcorn-demo::initialize-popcorn-demo)
  (roslisp:ros-info 'popcorn-demo "Popcorn-demo initialized.")

  (sleep 1)
  (roslisp:ros-info 'popcorn-demo "Waited 1 sec.")
  
  (navigate-to-stove  (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-center ))
  (roslisp:ros-info 'popcorn-demo "Reached the stove table front location.")
  
  (plan-library::grasp-object-single-arm :right
                                         kitchen-context::stove-table-drawer-right-designator)
  (roslisp:ros-info 'popcorn-demo "Grasped the right drawer's handle.")

  (sleep 10)
  
  (plan-library::ungrasp-object-single-arm
   :right
   kitchen-context::stove-table-drawer-right-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-drawer-right-designator :object-frame))
  (roslisp:ros-info 'popcorn-demo "Ungrasped the right drawer's handle.")
  
  (navigate-to-stove  (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :away-left))
  (roslisp:ros-info 'popcorn-demo "Reached the stove table front-left location.")

  (plan-library::dual-gripper-gripped kitchen-context::both-grippers-open-designator)
  )
  
                                         







