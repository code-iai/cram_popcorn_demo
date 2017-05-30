;;; Copyright (c) 2016 Gheorghe Lisca <lisca@cs.uni-bremen.de>
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

(in-package :kitchen-context)

;; (defparameter kitcken-island-designator
;;   (make-instance 'cram-designators:designator
;;     :description `((:object-type "kitchen_island")
;;                    (:object-frame ,*kitchen-island-frame*)
;;                    (:object-tf-frame-name ,*kitchen-island-frame-name*))))

(defparameter tool-holder-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "tool_holder")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#ToolHolder_fngh257tgh")
                   (:object-frame ,*tool-holder-frame*)
                   ;; (:object-tf-frame-name ,*tool-holder-frame-name*)
                   (:rack-0 *tool-holder-rack-0-frame*)
                   (:rack-0-above *tool-holder-rack-0-frame*)
                   (:rack-1 *tool-holder-rack-1-frame*)
                   (:rack-1-above *tool-holder-rack-1-frame*))))

(defparameter tool-holder-rack-0-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "tool_holder_rack_0")
                   (:object-frame ,*tool-holder-rack-0-frame*))))

(defparameter tool-holder-rack-1-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "tool_holder_rack_1")
                   (:object-frame ,*tool-holder-rack-1-frame*))))

(defparameter spoon-rounded-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "spoon_rounded")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#RoundSpoon_fngh257tgh")
                   (:object-frame ,*spoon-rounded-frame*)
                   ;; (:object-tf-frame-name ,*spoon-rounded-frame-name*)                   
                   (:object-grasping-frame ,*spoon-rounded-grasping-frame*)
                   (:object-pre-grasping-frame ,*spoon-rounded-pre-grasping-frame*)
                   (:object-end-effector-frame ,*spoon-rounded-end-effector-frame*)
                   (:object-initial-location-frame ,*spoon-rounded-initial-location-frame*))))

(defparameter spoon-flatten-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "spoon_flatten")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#FlatSpoon_fngh257tgh")
                   (:object-frame ,*spoon-flatten-frame*)
                   ;; (:object-tf-frame-name ,*spoon-flatten-frame-name*)                   
                   (:object-grasping-frame ,*spoon-flatten-grasping-frame*)
                   (:object-pre-grasping-frame ,*spoon-flatten-pre-grasping-frame*)
                   (:object-end-effector-frame ,*spoon-flatten-end-effector-frame*)
                   (:object-initial-location-frame ,*spoon-flatten-initial-location-frame*))))

(defparameter red-bowl-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "red_bowl")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#RedBowl_fngh257tgh")
                   (:object-frame ,*red-bowl-frame*)
                   (:over ,*red-bowl-over-frame*))))

(defparameter yellow-bowl-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "yellow_bowl")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#YellowBowl_fngh257tgh")
                   (:object-frame ,*yellow-bowl-frame*)
                   (:over ,*yellow-bowl-over-frame*))))

(defparameter tray-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "tray")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#PizzaTray_fngh257tgh")
                   (:object-frame ,*tray-frame*)
                   (:over ,*tray-over-frame*)
                   (:away ,*tray-away-frame*)
                   (:zone-0 ,*tray-zone-0-frame*)
                   (:zone-1 ,*tray-zone-1-frame*)
                   (:zone-2 ,*tray-zone-2-frame*)
                   (:zone-3 ,*tray-zone-3-frame*)
                   (:zone-4 ,*tray-zone-4-frame*)
                   (:zone-5 ,*tray-zone-5-frame*))))

(defparameter tomato-sauce-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "tomato_sauce")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#TomatoSauce_fngh257tgh")
                   (:object-frame ,*tomato-sauce-frame*)
                   (:over ,*tomato-sauce-over-frame*))))

(defparameter cheese-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "cheese")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Cheese_fngh257tgh")
                   (:object-frame ,*cheese-frame*)
                   (:over ,*cheese-over-frame*))))

(defparameter pizza-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "pizza")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Pizza_fngh257tgh")
                   (:object-frame ,*pizza-frame*)
                   (:over ,*pizza-over-frame*))))


(defparameter pot-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "pot")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Pot_fngh257tgh")
                   (:object-initial-location-frame ,*pot-initial-location-frame*)
                   ;;  (:object-frame ,*pot-frame*)
                   (:object-body-frame ,*pot-frame*)
                   (:over ,*pot-over-frame*)
                   (:pouring-in ,*pot-pouring-in-frame*)
                   (:object-pre-grasping-left-frame ,*pot-pregrasping-left-frame*)
                   (:object-pre-grasping-right-frame ,*pot-pregrasping-right-frame*)
                   (:object-grasping-left-frame ,*pot-grasping-left-frame*)
                   (:object-grasping-right-frame ,*pot-grasping-right-frame*)
                   (:top-frame ,*pot-top-frame*)
                   (:shaking-1-frame ,*pot-shaking-1-frame*)
                   (:shaking-2-frame ,*pot-shaking-2-frame*)
                   (:shaking-3-frame ,*pot-shaking-3-frame*))))

(defparameter lid-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "lid")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Lid_fngh257tgh")
                   (:object-initial-location-frame ,*lid-initial-location-frame*)
                   ;;   (:object-frame ,*lid-frame*)
                   (:object-body-frame ,*lid-frame*)
                   (:over ,*lid-over-frame*)
                   (:object-pre-grasping-frame ,*lid-pregrasping-frame*)
                   (:object-grasping-frame ,*lid-grasping-frame*))))

(defparameter salt-cellar-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "salt_cellar")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Salt_cellar_fngh257tgh")
                   (:object-initial-location-frame ,*salt-cellar-initial-location-frame*)
                   ;;  (:object-frame ,*salt-cellar-frame*)
                   (:object-body-frame ,*salt-cellar-frame*)
                   (:over ,*salt-cellar-over-frame*)
                   (:object-pre-grasping-left-frame ,*salt-cellar-pre-grasping-left-frame*)
                   (:object-pre-grasping-right-frame ,*salt-cellar-pre-grasping-right-frame*)
                   (:object-grasping-left-frame ,*salt-cellar-grasping-left-frame*)
                   (:object-grasping-right-frame ,*salt-cellar-grasping-right-frame*)
                   (:object-grasping-frame ,*salt-cellar-front-grasping-frame*)
                   (:object-pre-grasping-frame ,*salt-cellar-front-pre-grasping-frame*)
                   (:salting-left-first-frame , *salting-left-first-frame*)
                   (:salting-right-first-frame , *salting-right-first-frame*)
                   (:salting-right-second-frame , *salting-right-second-frame*)
                   (:salting-right-third-frame , *salting-right-third-frame*))))

(defparameter small-bowl-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "small_bowl")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Small_bowl_fngh257tgh")
                   (:object-initial-location-frame ,*small-bowl-initial-location-frame*)
                   ;;  (:object-frame ,*small-bowl-frame*)
                   (:object-body-frame ,*small-bowl-frame*)
                   (:over ,*small-bowl-over-frame*)
                   (:object-pre-grasping-frame ,*small-bowl-pregrasping-frame*)
                   (:object-grasping-frame ,*small-bowl-grasping-frame*))))

(defparameter deep-plate-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "deep_plate")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Deep_plate_fngh257tgh")
                   (:object-initial-location-frame ,*deep-plate-initial-location-frame*)
                   ;;  (:object-frame ,*deep-plate-frame*)
                   (:object-body-frame ,*deep-plate-frame*)
                   (:over ,*deep-plate-over-frame*)
                   (:over-salting ,*deep-plate-over-salting-frame*)
                   (:object-pre-grasping-frame ,*deep-plate-pregrasping-frame*)
                   (:object-grasping-frame ,*deep-plate-grasping-frame*)
                   (:pouring-in-1, *deep-plate-pouring-in-1-frame*)
                   (:pouring-in-2, *deep-plate-pouring-in-2-frame*)
                   (:pouring-in-3, *deep-plate-pouring-in-3-frame*)
                   (:pouring-in-4, *deep-plate-pouring-in-4-frame*))))

(defparameter stove-table-drawer-left-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "stove_table_drawer_left")
                   (:name  "http://knowrob.org/kb/IAI-kitchen.owl#Stove_table_drawer_left_fngh257tgh")
                   (:object-initial-location-frame ,*stove-table-drawer-left-initial-location-frame*)
                   ;;  (:object-frame ,*stove-table-drawer-left-frame*)
                   (:object-body-frame ,*stove-table-drawer-left-frame*)
                   (:over ,*stove-table-drawer-left-over-frame*)
                   (:object-pre-grasping-frame ,*stove-table-drawer-left-pre-grasping-frame*)
                   (:object-grasping-frame ,*stove-table-drawer-left-grasping-frame*)
                   (:side ,:left))))

(defparameter stove-table-drawer-right-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "stove_table_drawer_right")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Stove_table_drawer_right_fngh257tgh")
                   (:object-initial-location-frame ,*stove-table-drawer-right-initial-location-frame*)
                   ;;  (:object-frame ,*stove-table-drawer-right-frame*)
                   (:object-body-frame ,*stove-table-drawer-right-frame*)
                   (:over ,*stove-table-drawer-right-over-frame*)
                   (:object-pre-grasping-frame ,*stove-table-drawer-right-pre-grasping-frame*)
                   (:object-grasping-frame ,*stove-table-drawer-right-grasping-frame*)
                   (:side ,:right))))

(defparameter stove-table-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "stove_table")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Stove_table_fngh257tgh")
                   (:object-initial-location-frame ,*stove-table-initial-location-frame*)
                   ;;  (:object-frame ,*stove-table-frame*)
                   (:object-body-frame ,*stove-table-frame*) 
                   (:away-center ,*stove-table-away-center-frame*)
                   (:away-left ,*stove-table-away-left-frame*)
                   (:away-right ,*stove-table-away-right-frame*)
                   (:close-to-center ,*stove-table-close-to-center-frame*)
                   (:close-to-left ,*stove-table-close-to-left-frame*)
                   (:close-to-right ,*stove-table-close-to-right-frame*) 
                   (:cooking-plate ,*stove-cooking-plate-frame*) 
                   (:cooking-plate-left ,*stove-cooking-plate-left-frame*)
                   (:cooking-plate-right ,*stove-cooking-plate-right-frame*)
                   (:over-cooking-plate-left ,*stove-over-cooking-plate-left-frame*) 
                   (:over-cooking-plate-right ,*stove-over-cooking-plate-right-frame*)                     
                   (:region-left ,*stove-region-left-frame*)
                   (:region-right ,*stove-region-right-frame*)
                   (:over-region-left ,*stove-over-region-left-frame*) 
                   (:over-region-right ,*stove-over-region-right-frame*)
                   (:region-left-corner ,*stove-region-left-corner-frame*)
                   (:region-left-corner-2 ,*stove-region-left-corner-2-frame*)
                   (:region-right-corner ,*stove-region-right-corner-frame*)
                   (:over-region-left-corner ,*stove-over-region-left-corner-frame*)
                   (:over-region-right-corner ,*stove-over-region-right-corner-frame*)
                   (:open-left-drawer ,*stove-open-drawer-left-frame*)
                   (:open-right-drawer ,*stove-open-drawer-right-frame*))))


(defparameter knob-on-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "knob_on")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Knob_fngh257tgh")
                   (:object-initial-location-frame ,*stove-knob-initial-location-frame*)
                   ;;  (:object-frame ,*stove-knob-on-object-frame*)
                   (:object-body-frame ,*stove-knob-on-object-frame*)
                   (:object-pre-grasping-frame  ,*stove-knob-pregrasping-on-frame*) 
                   (:object-grasping-frame  ,*stove-knob-grasping-on-frame*)
                   (:knob-on ,*stove-knob-on-frame*))))

(defparameter knob-off-designator
  (make-instance 'cram-designators:designator
    :description `((:object-type "knob_off")
                   (:name "http://knowrob.org/kb/IAI-kitchen.owl#Knob_fngh257tgh")
                   (:object-initial-location-frame ,*stove-knob-initial-location-frame*)
                   ;;  (:object-frame ,*stove-knob-off-object-frame*)
                   (:object-body-frame ,*stove-knob-off-object-frame*)
                   (:object-pre-grasping-frame  ,*stove-knob-pregrasping-off-frame*) 
                   (:object-grasping-frame  ,*stove-knob-grasping-off-frame*)
                   (:knob-off ,*stove-knob-off-frame*))))


                  


