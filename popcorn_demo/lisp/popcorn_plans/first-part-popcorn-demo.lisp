;;; Copyright (c) 2017 Mihaela Popescu <popescu@cs.uni-bremen.de>
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


(defparameter *stove-table-away-left-navigation*
  (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :away-left))

(defparameter *stove-table-away-right-navigation*
  (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :away-right))

(defparameter *stove-table-close-to-left-navigation*
  (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-left))

(defparameter *stove-table-close-to-right-navigation*
  (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-right))

(defparameter *stove-table-close-to-center-navigation*
  (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-center))

(defparameter *open-drawer* "open")
(defparameter *close-drawer* "close")



;;; ================================================================================== ;;;
;;; manipulate-drawer (drawer-designator open-or-close)                                ;;;
;;; ================================================================================== ;;;

(defun manipulate-drawer (drawer-designator open-or-close)

  ;;navigate in front of the table
  (roslisp:ros-info 'open-drawer "Waiting to navigate in front of the drawer...")
  (plan-library::navigate-at-location  *stove-table-close-to-center-navigation*)

  ;;grasp drawer handle
  (roslisp:ros-info 'open-drawer "Waiting to grasp the drawer handle...")
  (plan-library::grasp-object-single-arm (cram-designators:desig-prop-value drawer-designator :side) drawer-designator)

  (let ((drawer-final-location-designator
          ;;if the drawer needs to be open
          (cond ((equal open-or-close "open")
                 (cond ((equal (cram-designators:desig-prop-value drawer-designator :side) :left)
                        (tf-utilities::designator-property-as-location-designator  kitchen-context::stove-table-designator :open-left-drawer))
                       ((equal (cram-designators:desig-prop-value drawer-designator :side) :right)
                        (tf-utilities::designator-property-as-location-designator  kitchen-context::stove-table-designator :open-right-drawer))))

                ;;if the drawer needs to be closed
                ((equal open-or-close "close")
                 (tf-utilities::designator-property-as-location-designator drawer-designator :object-initial-location-frame)))))
    
    ;;move drawer at location
    (roslisp:ros-info 'open-drawer "Waiting to move drawer at the new location...")
    (plan-library::single-arm-move-object-at-location
     (cram-designators:desig-prop-value drawer-designator :side)
     drawer-designator
     drawer-final-location-designator)

    ;;ungrasp drawer
    (roslisp:ros-info 'open-drawer "Waiting to ungrasp the drawer handle...")
    (plan-library::ungrasp-object-single-arm
     (cram-designators:desig-prop-value drawer-designator :side)
     drawer-designator
     drawer-final-location-designator)))



;;; ================================================================================== ;;;
;;; first-part-popcorn-demo ()                                                         ;;;
;;; ================================================================================== ;;;

(defun first-part-popcorn-demo ()

  ;; Initialize robot
  ;; Move both arms away from robot's field of view.
  (roslisp:ros-info 'first-part-popcorn-demo "Moving both arms away from robot's field of view...")
  (plan-library:dual-arm-move-at-locations
   designator-library::pr2-ready-arm-left-at-chest-designator
   designator-library::pr2-ready-arm-right-at-chest-designator)

  ;; Open both grippers
  (roslisp:ros-info 'first-part-popcorn-demo "Opening both grippers...")
  (plan-library::dual-gripper-gripped
   kitchen-context::both-grippers-open-designator)
  
  ;; Navigate at the location from which it is easy to grasp objects from table.
  (roslisp:ros-info 'first-part-popcorn-demo "Navigating in front-right of the table...")
  (plan-library::navigate-at-location  *stove-table-close-to-right-navigation*)

  ;; Detect the pot using perception
  (plan-library::detect-object  kitchen-context::pot-designator)
  
  ;; Grasp the pot
  (roslisp:ros-info 'first-part-popcorn-demo "Grasping the pot...")
  (plan-library::grasp-object-dual-arm
   kitchen-context::pot-designator
   kitchen-context::both-grippers-close-designator)

  ;; Move pot over the stove table
  (roslisp:ros-info 'first-part-popcorn-demo "Moving the pot over the stove table...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :over-region-right))

  ;; Navigate at the location close-to-center of the stove
  (roslisp:ros-info 'first-part-popcorn-demo "Navigating close to the center of the stove table...")
  (plan-library::navigate-at-location  *stove-table-close-to-center-navigation*)

  ;; Move pot on the left burner
  (roslisp:ros-info 'first-part-popcorn-demo "Moving the pot on the left burner...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-left))
  
  ;; Ungrasp the pot
  (roslisp:ros-info 'first-part-popcorn-demo "Ungrasping the pot...")
  (plan-library::ungrasp-object-dual-arm
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-left)
   kitchen-context::both-grippers-open-designator)

  ;; Open right drawer
  (roslisp:ros-info 'first-part-popcorn-demo "Opening the right drawer...")
  (manipulate-drawer kitchen-context::stove-table-drawer-right-designator *open-drawer*)

  ;; Detect the small bowl using perception
  (plan-library::detect-object  kitchen-context::small-bowl-designator)
  
  ;; Grasp the small bowl
  (roslisp:ros-info 'first-part-popcorn-demo "Grasping the small bowl...")
  (plan-library::grasp-object-single-arm
   :right 
   kitchen-context::small-bowl-designator)

   ;; Detect the pot using perception
  (plan-library::detect-object  kitchen-context::pot-designator)

  ;; Move the small bowl over the pot, pour the content and place it back in the drawer
  (roslisp:ros-info 'first-part-popcorn-demo "Moving the small bowl over the pot, pouring the content and placing it back...")
  (plan-library::single-arm-move-object-at-location
   :right
   kitchen-context::small-bowl-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::small-bowl-designator :object-initial-location-frame)
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-drawer-right-designator :over)
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :over)
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :pouring-in)
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-drawer-right-designator :over))

  ;; Ungrasp the small bowl 
  (roslisp:ros-info 'first-part-popcorn-demo "Ungrasping the small bowl...")
  (plan-library::ungrasp-object-single-arm
   :right
   kitchen-context::small-bowl-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::small-bowl-designator :object-initial-location-frame))

  ;; Detect the lid using perception
  (plan-library::detect-object  kitchen-context::lid-designator)
  
  ;; Grasp the lid
  (roslisp:ros-info 'first-part-popcorn-demo "Grasping the lid...")
  (plan-library::grasp-object-single-arm
   :right 
   kitchen-context::lid-designator)

  ;; Move the lid over the drawer
  (roslisp:ros-info 'first-part-popcorn-demo "Moving the lid over the drawer...")
  (plan-library::single-arm-move-object-at-location
   :right
   kitchen-context::lid-designator (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-drawer-right-designator :over))
  
  ;; Detect the pot using perception
  (plan-library::detect-object  kitchen-context::pot-designator)
  
  ;; Navigate at the location close-to-left of the stove
  (roslisp:ros-info 'first-part-popcorn-demo "Navigating close to the left of the stove table...")
  (plan-library::navigate-at-location  *stove-table-close-to-left-navigation*)

  ;; Move the lid on the pot
  (roslisp:ros-info 'first-part-popcorn-demo "Moving the lid on the pot...")
  (plan-library::single-arm-move-object-at-location
   :right
   kitchen-context::lid-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :top-frame)
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :over))

  ;; Ungrasp the lid
  (roslisp:ros-info 'first-part-popcorn-demo "Ungrasping the lid...")
  (plan-library::ungrasp-object-single-arm
   :right
   kitchen-context::lid-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :top-frame))

  ;; Navigate at the location close-to-center of the stove
  (roslisp:ros-info 'first-part-popcorn-demo "Navigating close to the center of the stove table...")
  (plan-library::navigate-at-location  *stove-table-close-to-center-navigation*)

  ;; Close the right drawer
  (roslisp:ros-info 'first-part-popcorn-demo "Close right drawer...")
  (manipulate-drawer kitchen-context::stove-table-drawer-right-designator *close-drawer*)

  ;; Detect the pot using perception
  (plan-library::detect-object  kitchen-context::pot-designator)
  
  ;; Grasp the pot to shake it
  (roslisp:ros-info 'first-part-popcorn-demo "Grasping the pot to shake it...")
  (plan-library::grasp-object-dual-arm
   kitchen-context::pot-designator
   kitchen-context::both-grippers-close-designator)

  ;; Shake the pot
  (roslisp:ros-info 'first-part-popcorn-demo "Shaking the pot...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-left)
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :over-cooking-plate-left)
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :shaking-2-frame)
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :shaking-1-frame)
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :shaking-3-frame))
  
  ;; Ungrasp the pot
  (roslisp:ros-info 'first-part-popcorn-demo "Ungrasping the pot...")
  (plan-library::ungrasp-object-dual-arm
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-left)
   kitchen-context::both-grippers-open-designator)

  ;; Grasp the knob
  (roslisp:ros-info 'popcorn-demo "Grasping the knob...")                                                          
  (plan-library::grasp-object-single-arm :right kitchen-context::knob-on-designator)                                  

  ;; Switch the knob on
  (roslisp:ros-info 'popcorn-demo "Switching the knob ON...")                                                     
  (plan-library::singular-arm-move-at-location                                                                  
   :right                                                                                                       
   (tf-utilities::designator-property-as-location-designator kitchen-context::knob-on-designator :knob-on))   

  ;; Ungrasp the knob
  (roslisp:ros-info 'popcorn-demo "Ungrasping the knob...")                                                        
  (plan-library::ungrasp-object-single-arm                                                                      
   :right                                                                                               
   kitchen-context::knob-off-designator                                                                        
   (tf-utilities::designator-property-as-location-designator kitchen-context::knob-on-designator :knob-on ))

  ;; Open left drawer
  (roslisp:ros-info 'first-part-popcorn-demo "Opening the left drawer...")
  (manipulate-drawer kitchen-context::stove-table-drawer-left-designator *open-drawer*)

  ;;; Detect the deep plate using perception
  (plan-library::detect-object  kitchen-context::deep-plate-designator)
  
  ;; Grasp the deep plate
  (roslisp:ros-info 'first-part-popcorn-demo "Grasping the deep plate...")
  (plan-library::grasp-object-single-arm
   :left 
   kitchen-context::deep-plate-designator)

  ;; Move the deep plate on the table
  (roslisp:ros-info 'first-part-popcorn-demo "Moving the deep plate on the table...")
  (plan-library::single-arm-move-object-at-location
   :left
   kitchen-context::deep-plate-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :region-left-corner)
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-drawer-left-designator :over)
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :over-region-left-corner))

  ;; Ungrasp the deep plate
  (roslisp:ros-info 'first-part-popcorn-demo "Ungrasping the deep plate...")
  (plan-library::ungrasp-object-single-arm
   :left
   kitchen-context::deep-plate-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :region-left-corner))
  
  ;; Close left drawer
  (roslisp:ros-info 'first-part-popcorn-demo "Close left drawer...")
  (manipulate-drawer kitchen-context::stove-table-drawer-left-designator *close-drawer*)

  ;; Navigate away to the centre of the table 
  (roslisp:ros-info 'first-part-popcorn-demo "Navigating away of the table centre...")
  (plan-library::navigate-at-location
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :away-center))

  ;; Wait for the popcorn to pop
  (roslisp:ros-info 'first-part-popcorn-demo "Waiting for the popcorn to pop...")
  (sleep (* 0 1))

  ;; End of first part
  (roslisp:ros-info 'first-part-popcorn-demo "End of first part..."))
