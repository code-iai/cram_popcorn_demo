;; Copyright (c) 2017 Boglarka Erdodi <erdoedib@uni-bremen.de>
;; All rights reserved.

;; Redistribution and use in source and binary forms, with or without
;; modification, are permitted provided that the following conditions are met:

;;     * Redistributions of source code must retain the above copyright
;;       notice, this list of conditions and the following disclaimer.
;;     * Redistributions in binary form must reproduce the above copyright
;;       notice, this list of conditions and the following disclaimer in the
;;       documentation and/or other materials provided with the distribution.
;;     * Neither the name of Gheorghe Lisca nor the names of his
;;       partners may be used to endorse or promote products derived from
;;       this software without specific prior written permission.

;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;; POSSIBILITY OF SUCH DAMAGE.



(in-package :popcorn-demo)

;;;========================================================================== ;;;
;;;update objects for second part popcorn demo                                ;;;
;;;========================================================================== ;;;


(defun update-objects-for-second-part-popcorn-demo ()

 ;;; update pot's location (plite)
  (tf-utilities::update-object-location
   kitchen-context::pot-designator   
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-left)
   (cl-transforms:make-identity-transform))

 ;;; update lid's location (pot)
  (tf-utilities::update-object-location
   kitchen-context::lid-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::pot-designator :top-frame)
   (cl-transforms:make-identity-transform))

 ;;; update deep plate location (stove table left side)
  (tf-utilities::update-object-location
   kitchen-context::deep-plate-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :region-left-corner)
   (cl-transforms:make-identity-transform)))

;;;========================================================================== ;;;
;;;second-part-popcorn-demo                                                   ;;;
;;;========================================================================== ;;;


(defun second-part-popcorn-demo ()

  ;; (roslisp:ros-info 'popcorn-demo "Ros-node starting...")
  ;; (roslisp-utilities:startup-ros)                                                                
  
  ;; (Roslisp:ros-info 'popcorn-demo "Popcorn-demo initializing...")
  ;; (popcorn-demo::initialize-popcorn-demo)
  
  ;; (roslisp:ros-info 'popcorn-demo "Waiting 1 sec...")
  ;; (sleep 1)
  
  ;; (roslisp:ros-info 'second-part-popcorn-demo "Objects updating for the second part popcorn-demo...")
  ;; (update-objects-for-second-part-popcorn-demo)
  

  (roslisp:ros-info 'second-part-popcorn-demo "Reaching the stove table close-to-center location...")
  (plan-library::navigate-at-location
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-center))

  (roslisp:ros-info 'second-part-popcorn-demo "Opening dual gripper...")
  (plan-library::dual-gripper-gripped
   kitchen-context::both-grippers-open-designator)

  
  (roslisp:ros-info 'second-part-popcorn-demo "Grasping the knob...")                                                         
  (plan-library::grasp-object-single-arm
   :right                                                                 
   kitchen-context::knob-off-designator)                                  
  
  (roslisp:ros-info 'second-part-popcorn-demo "Switching the knob off...")                                                 
  (plan-library::singular-arm-move-at-location                                                                  
   :right                                                                                                       
   (tf-utilities::designator-property-as-location-designator kitchen-context::knob-off-designator :knob-off))   
  
  (roslisp:ros-info 'second-part-popcorn-demo "Ungrasping the knob...")                                                    
  (plan-library::ungrasp-object-single-arm                                                                      
   :right                                                                                                       
   kitchen-context::knob-off-designator                                                                        
   (tf-utilities::designator-property-as-location-designator kitchen-context::knob-off-designator :knob-off))

  ;;; Detect the pot using perception
  (plan-library::detect-object  kitchen-context::pot-designator)
  
  (roslisp:ros-info 'second-part-popcorn-demo "Grasping the pot's handles with dual arm.")
  (plan-library::grasp-object-dual-arm 
   kitchen-context::pot-designator
   kitchen-context::both-grippers-close-designator)  

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the pot to the cooking-plate-right-frame...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-right))
  
  (roslisp:ros-info 'second-part-popcorn-demo "Ungrasping the pot's handles with dual arm.")
  (plan-library::ungrasp-object-dual-arm 
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-right)
   kitchen-context::both-grippers-open-designator)

  ;;; Detect the lid using perception
  (plan-library::detect-object  kitchen-context::lid-designator)
  
  (roslisp:ros-info 'second-part-popcorn-demo "Grasping the lid...")
  (plan-library::grasp-object-single-arm
   :right
   kitchen-context::lid-designator)
  
  (roslisp:ros-info 'second-part-popcorn-demo "Moving the lid in the right part of the stove table...")
  (plan-library::single-arm-move-object-at-location
   :right
   kitchen-context::lid-designator 
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :region-right-corner)
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :over-cooking-plate-right)
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :over-region-right-corner))

  (roslisp:ros-info 'second-part-popcorn-demo "Ungrasping the lid...")                                                        
  (plan-library::ungrasp-object-single-arm                                                                      
   :right                                                                                                       
   kitchen-context::lid-designator                                                                         
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :region-right-corner))

   ;;; Detect the pot using perception
  (plan-library::detect-object  kitchen-context::pot-designator)
  
  (roslisp:ros-info 'second-part-popcorn-demo "Grasping the pot's handles with dual arm.")
  (plan-library::grasp-object-dual-arm 
   kitchen-context::pot-designator
   kitchen-context::both-grippers-close-designator)
  
  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the pot over the cooking-plate-right-frame...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :over-cooking-plate-right ))
  
  (roslisp:ros-info 'second-part-popcorn-demo "Reaching the stove table close-to-left location...")
  (plan-library::navigate-at-location
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-left))

  ;; Detect the deep plate using perception
  (plan-library::detect-object  kitchen-context::deep-plate-designator)

  
  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the pot to the over-deep-plate, pouring-in-1, pouring-in-2, pouring-in-3, pouring-in-4...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :pouring-in-4)
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :over )
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :pouring-in-1)
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :pouring-in-2)
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :pouring-in-3))

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the pot to the pouring-in-3, pouring-in-2, pouring-in-1 ...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :over)
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :pouring-in-3)
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :pouring-in-2)
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :pouring-in-1))
  
  
  (roslisp:ros-info 'second-part-popcorn-demo "Reaching the stove table close-to-center location...")
  (plan-library::navigate-at-location
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-center ))

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the pot on  the cooking plate right frame...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-right))
  
  (roslisp:ros-info 'second-part-popcorn-demo "Ungrasping the pot's handles with dual arm...")
  (plan-library::ungrasp-object-dual-arm 
   kitchen-context::pot-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :cooking-plate-right)
   kitchen-context::both-grippers-open-designator)

  (roslisp:ros-info 'second-part-popcorn-demo "Reaching the stove table close-to-left location...")
  (plan-library::navigate-at-location
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :close-to-left ))

  ;;; Detect the salt-cellar using perception
  (plan-library::detect-object  kitchen-context::salt-cellar-designator)
  
  (roslisp:ros-info 'second-part-popcorn-demo "Grasping the salt cellar with right arm...")
  (plan-library::grasp-object-single-arm
   :right
   kitchen-context::salt-cellar-designator) 

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt cellar to the deep-plate-over-frame...")
  (plan-library::double-arm-move-object-at-location
   kitchen-context::salt-cellar-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::deep-plate-designator :over-salting))

  (roslisp:ros-info 'popcorn-demo "Waiting 1 sec...")
  (sleep 1)

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the left gripper to salting-left-first-frame ...")
  (plan-library::singular-arm-move-at-location 
   :left
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-left-first-frame))

  (roslisp:ros-info 'second-part-popcorn-demo "Closing dual gripper (this is for the left gripper, the right is already closed...")
  (plan-library::singular-gripper-gripped
   :left
   0.000
   60.000)
  
  (tf-utilities::update-object-location
   kitchen-context::salt-cellar-designator
   plan-library::in-robot-left-gripper-designator
   (cl-transforms:transform-inv (cram-designators:desig-prop-value kitchen-context::salt-cellar-designator :object-grasping-left-frame)))

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-first-frame ...")
  (plan-library::singular-arm-move-at-location
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-first-frame))

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-second-frame ...")
  (plan-library::singular-arm-move-at-location 
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-second-frame))

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-third-frame ...")
  (plan-library::singular-arm-move-at-location 
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-third-frame))

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-second-frame ...")
  (plan-library::singular-arm-move-at-location 
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-second-frame))

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-first-frame ...")
  (plan-library::singular-arm-move-at-location
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-first-frame)) 

(roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-second-frame ...")
  (plan-library::singular-arm-move-at-location 
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-second-frame))

 (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-third-frame ...")
  (plan-library::singular-arm-move-at-location 
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-third-frame))

(roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-second-frame ...")
  (plan-library::singular-arm-move-at-location 
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-second-frame))

(roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to salting-first-frame ...")
  (plan-library::singular-arm-move-at-location
   :right
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :salting-right-first-frame)) 

  (tf-utilities::update-object-location
   kitchen-context::salt-cellar-designator
   plan-library::in-robot-right-gripper-designator
   (cl-transforms:transform-inv (cram-designators:desig-prop-value kitchen-context::salt-cellar-designator :object-grasping-right-frame)))


  (roslisp:ros-info 'second-part-popcorn-demo "Opening left gripper ...")
  (plan-library::singular-gripper-gripped
   :left
   0.070
   60.000)

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the left gripper to pre grasping left frame ...")
  (plan-library::singular-arm-move-at-location 
   :left
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :object-pre-grasping-left-frame))

  (roslisp:ros-info 'second-part-popcorn-demo "Waiting to move the salt-cellar to the initial location frame ...")
  (plan-library::single-arm-move-object-at-location
   :right
   kitchen-context::salt-cellar-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator :object-initial-location-frame))

  (roslisp:ros-info 'second-part-popcorn-demo "Ungrasping the salt-cellar with right arm...")
  (plan-library::ungrasp-object-single-arm
   :right 
   kitchen-context::salt-cellar-designator
   (tf-utilities::designator-property-as-location-designator kitchen-context::salt-cellar-designator  :object-initial-location-frame ))

  ;; Detect the deep plate using perception
  (plan-library::detect-object  kitchen-context::deep-plate-designator)

  (roslisp:ros-info 'second-part-popcorn-demo "Grasping the deep plate with the popcorn...")
  (plan-library::grasp-object-single-arm
   :left
   kitchen-context::deep-plate-designator)
 
  (roslisp:ros-info 'second-part-popcorn-demo "Moving the deep-plate to the  over-region-left frame...")
  (plan-library::single-arm-move-object-at-location
   :left
   kitchen-context::deep-plate-designator 
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :over-region-left ))

  (roslisp:ros-info 'popcorn-demo "Waiting 15  sec...")
  (sleep 15)


    (roslisp:ros-info 'second-part-popcorn-demo "Moving the deep-plate to the region-left-corner frame...")
  (plan-library::single-arm-move-object-at-location
   :left
   kitchen-context::deep-plate-designator 
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :region-left-corner ))
  
  (roslisp:ros-info 'second-part-popcorn-demo "Ungrasping the deep-plate ...")                                             
  (plan-library::ungrasp-object-single-arm                                                                      
   :left                                                                                                       
   kitchen-context::deep-plate-designator                                                                         
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :region-left-corner))

  (roslisp:ros-info 'second-part-popcorn-demo "Reaching the stove table close-to-center location...")
  (plan-library::navigate-at-location
   (tf-utilities::designator-property-as-location-designator kitchen-context::stove-table-designator :away-left))

  (roslisp:ros-info 'second-part-popcorn-demo "End of second-part-popcorn-demo ..."))

