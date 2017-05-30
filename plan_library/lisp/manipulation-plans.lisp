;;; Copyright (c) 2013-16 Gheorghe Lisca <lisca@cs.uni-bremen.de>
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

;;
;; boxy specific location frames
;;

;; (defparameter in-robot-left-gripper-designator designator-library::in-boxy-left-gripper-designator)
;; (defparameter in-robot-right-gripper-designator designator-library::in-boxy-right-gripper-designator)

;; (defparameter ready-arm-left-designator designator-library::boxy-ready-arm-left-designator)
;; (defparameter ready-arm-right-designator designator-library::boxy-ready-arm-right-designator)

;; (defparameter ready-arm-left-at-chest-designator designator-library::boxy-ready-arm-left-at-chest-designator)
;; (defparameter ready-arm-right-at-chest-designator designator-library::boxy-ready-arm-right-at-chest-designator)

;;
;; pr2 specific location frames
;;

(defparameter in-robot-left-gripper-designator designator-library::in-pr2-left-gripper-designator)
(defparameter in-robot-right-gripper-designator designator-library::in-pr2-right-gripper-designator)

(defparameter ready-arm-left-designator designator-library::pr2-ready-arm-left-designator)
(defparameter ready-arm-right-designator designator-library::pr2-ready-arm-right-designator)

(defparameter ready-arm-left-at-chest-designator designator-library::pr2-ready-arm-left-at-chest-designator)
(defparameter ready-arm-right-at-chest-designator designator-library::pr2-ready-arm-right-at-chest-designator)

;;; ========================================================================================= ;;;
;;; grab-object-from-location (arm object-designator location-designator grasping-designator) ;;;
;;; ========================================================================================= ;;;

(defun grab-object-from-location (arm object-designator location-designator grasping-designator)

  (let ((log-node-id (beliefstate:start-node "GRASP-OBJECT" nil)))

    (cond ((eq arm :left)
           (beliefstate::annotate-resource "bodyPartsUsed" "http://knowrob.org/kb/Boxy.owl#boxy_left_gripper" "knowrob"))
          ((eq arm :right)
           (beliefstate::annotate-resource "bodyPartsUsed" "http://knowrob.org/kb/Boxy.owl#boxy_right_gripper" "knowrob"))
          (t
           (roslisp:ros-warn (grab-object-from-location) "the received arm ~A is invalid ... !" arm)))
    
    (beliefstate::annotate-resource "objectActedOn" (cram-designators:desig-prop-value object-designator :name) "knowrob")

    (roslisp:ros-info
     (grab-object-from-location) "start grabbing object ~A with gripper ~A from location ..."
     (cram-designators::desig-prop-value object-designator :object-type) arm)

    ;; sight towards object's expected location.
    (plan-library:sighted location-designator)      
    (roslisp:ros-info (grab-object-from-location) "sighted towards the objects's location expected location")
    
    ;; recognize object's part to be manipulated.
    (plan-library:visualized object-designator)
    (roslisp:ros-info (grab-object-from-location) "recognized the object ")

    ;; annotate the opening of the gripper
    (let ((log-node-id (beliefstate:start-node "OPEN-GRIPPER")))
      
      ;; open the gripper to be used.      
      (plan-library:singular-gripper-gripped arm 25.0 40.0)
      (roslisp:ros-info (grab-object-from-location) "opened the ~A gripper ..." arm)

      (beliefstate:stop-node log-node-id :success T))

    ;; annotate the reaching for the object
    (let ((log-node-id (beliefstate:start-node "REACH")))

      (beliefstate::annotate-resource "goalLocation" (cram-designators:desig-prop-value object-designator :name) "knowrob")

      ;; grasp object's part be moved.
      (plan-library::singular-arm-reach-in
       arm (designator-library::particularise-action-designator-for-object-designator
            grasping-designator object-designator))

      (beliefstate:stop-node log-node-id :success T))

    ;; annotate the closing of the gripper
    (let ((log-node-id (beliefstate:start-node "CLOSE-GRIPPER")))
      
      ;; close the gripper.
      (plan-library:singular-gripper-gripped arm 8.0 80.0)

      (beliefstate:stop-node log-node-id :success T))
    
    ;; the object is relative to robot's gripper.
    (tf-utilities::update-object-location
     object-designator
     (cond ((eq arm :left) in-robot-left-gripper-designator)
           ((eq arm :right) in-robot-right-gripper-designator))   
     (cl-transforms:transform-inv
      (cram-designators:desig-prop-value
       (designator-library::particularise-action-designator-for-object-designator
        grasping-designator object-designator) :grasping-position)))
    
    (roslisp:ros-info
     (grab-object-from-location) "the object ~A is relative to the ~A gripper ..."
     (cram-designators::desig-prop-value object-designator :object-type) arm)
    
    (roslisp:ros-info
     (grab-object-from-location) "finished grabbing object ~A with gripper ~A from location ..."
     (cram-designators::desig-prop-value object-designator :object-type) arm)
    
    (beliefstate:stop-node log-node-id :success T)))


;;; ======================================================================================== ;;;
;;; place-object-at-location (arm object-designator location-designator grasping-designator) ;;;
;;; ======================================================================================== ;;;

(defun place-object-at-location (arm object-designator location-designator grasping-designator)
  
  (let ((log-node-id (beliefstate:start-node "PUT-DOWN-OBJECT" nil)))

    (cond ((eq arm :left)
           (beliefstate::annotate-resource "bodyPartsUsed" "http://knowrob.org/kb/Boxy.owl#boxy_left_gripper" "knowrob"))
          ((eq arm :right)
           (beliefstate::annotate-resource "bodyPartsUsed" "http://knowrob.org/kb/Boxy.owl#boxy_right_gripper" "knowrob"))
          (t
           (roslisp:ros-warn (grab-object-from-location) "the received arm ~A is invalid ... !" arm)))

    (beliefstate::annotate-resource
     "objectActedOn" (cram-designators:desig-prop-value object-designator :name) "knowrob")

    (roslisp:ros-info (place-object-at-location) "start placing object at location into location ...")

    ;; sight towards object's expected location.
    (plan-library:sighted location-designator)      
    (roslisp:ros-info (grab-object-from-location) "sighted towards the objects's location expected location")
    
    ;; move the object into the destination location.
    (let ((object-designator-projected
            (tf-utilities::start-projecting-object-at-location object-designator location-designator)))
      
      (plan-library:singular-arm-move-at-location
       arm (tf-utilities::designator-property-as-location-designator
            (designator-library::particularise-action-designator-for-object-designator
             grasping-designator object-designator-projected) :grasping-position))
      
      (roslisp:ros-info (place-container-at-location) "moved container into the destination location ..."))

    ;; update container's location relative to the destination location.
    (tf-utilities::update-object-location
     object-designator location-designator (cl-transforms:make-identity-transform))
    (roslisp:ros-info (place-object-at-location) "the container is relative to the destination location ...")

    ;; annotate the opening of the gripper
    (let ((log-node-id (beliefstate:start-node "OPEN-GRIPPER")))

      ;; open the gripper.
      (plan-library:singular-gripper-gripped arm 30.0 60.000)
      
      (roslisp:ros-info (place-object-at-location) "opened ~A gripper ..." arm)

      (beliefstate:stop-node log-node-id :success T))

    ;; annotate the releasing of the object
    (let ((log-node-id (beliefstate:start-node "RELEASE")))

      (plan-library::singular-arm-reach-out
       arm (designator-library::particularise-action-designator-for-object-designator
            grasping-designator object-designator))
      
      (beliefstate:stop-node log-node-id :success T))

    (roslisp:ros-info (place-object-at-location) "finished placing object at location ...")

    (beliefstate:stop-node log-node-id :success T)))

;;; ======================================================================================= ;;;
;;; move-object-at-location (arm object-designator location-designator grasping-designator) ;;;
;;; ======================================================================================= ;;;

(defun move-object-at-location (arm object-designator location-designator grasping-designator)

  ;; annotate the motion of the object at a specific location
  (let ((log-node-id (beliefstate:start-node "MOVE-OBJECT-AT-LOCATION" nil)))

    (cond ((eq arm :left)
           (beliefstate::annotate-resource "bodyPartsUsed" "http://knowrob.org/kb/Boxy.owl#boxy_left_gripper" "knowrob"))
          ((eq arm :right)
           (beliefstate::annotate-resource "bodyPartsUsed" "http://knowrob.org/kb/Boxy.owl#boxy_right_gripper" "knowrob"))
          (t
           (roslisp:ros-warn (grab-object-from-location) "the received arm ~A is invalid ... !" arm)))

    (beliefstate::annotate-resource
     "objectActedOn" (cram-designators:desig-prop-value object-designator :name) "knowrob")

    (roslisp:ros-info (move-object-at-location) "start moving the object into destination location ...")
    
    ;; sight towards object destination location
    (plan-library:sighted location-designator)
    (roslisp:ros-info (move-object-at-location) "sighted towards object destination location ...")
    
    (let ((object-designator-projected
            (tf-utilities::start-projecting-object-at-location object-designator location-designator)))
      
      ;; move the object into the specified location.
      (plan-library:singular-arm-move-at-location
       arm (tf-utilities::designator-property-as-location-designator
            (designator-library::particularise-action-designator-for-object-designator
             grasping-designator object-designator-projected) :grasping-position)))

    (roslisp:ros-info (move-object-at-location) "finished moving the object into destination location ...")

    (beliefstate:stop-node log-node-id :success T)))


;;; ==================================================================================================== ;;;
;;; move-object-end-effector-at-location (arm object-designator location-designator grasping-designator) ;;;
;;; ==================================================================================================== ;;;

(defun move-object-end-effector-at-location (arm object-designator location-designator grasping-designator)
  
  (let ((log-node-id (beliefstate:start-node "MOVE-OBJECT-AT-LOCATION" nil)))

    (cond ((eq arm :left)
           (beliefstate::annotate-resource "bodyPartsUsed" "http://knowrob.org/kb/Boxy.owl#boxy_left_gripper" "knowrob"))
          ((eq arm :right)
           (beliefstate::annotate-resource "bodyPartsUsed" "http://knowrob.org/kb/Boxy.owl#boxy_right_gripper" "knowrob"))
          (t
           (roslisp:ros-warn (grab-object-from-location) "the received arm ~A is invalid ... !" arm)))

    (beliefstate::annotate-resource
     "objectActedOn" (cram-designators:desig-prop-value object-designator :name) "knowrob")

    (roslisp:ros-info (move-object-end-effector-at-location) "start moving object's end effector into location ...")
    
    ;; sight towards object's expected location.
    (plan-library:sighted location-designator)
    (roslisp:ros-info (move-object-end-effector-at-location) "sighted towards the location ...")
    
    (let* ((object-end-effector-inv-offset
             (cl-transforms:transform-inv
              (cram-designators:desig-prop-value object-designator :object-end-effector-frame)))
           (object-designator-projected
             (tf-utilities::start-projecting-object-at-location
              object-designator location-designator object-end-effector-inv-offset)))

      ;; move object's end effectot into the specified location.
      (plan-library:singular-arm-move-at-location
       arm
       (tf-utilities::designator-property-as-location-designator
        (designator-library::particularise-action-designator-for-object-designator
         grasping-designator object-designator-projected) :grasping-position)))

    (roslisp:ros-info (move-object-end-effector-at-location) "finished moving object's end effector into location ...")

    (beliefstate:stop-node log-node-id :success T)))

;;; ============================================================================================================= ;;;
;;; pour-from-object-into-object (arm source-object-designator destination-object-designator grasping-designator) ;;;
;;; ============================================================================================================= ;;;

(defun pour-from-object-into-object
    (arm source-object-designator destination-object-designator grasping-designator)

  (let ((log-node-id (beliefstate:start-node "POURING" nil)))

    (beliefstate:add-designator-to-node source-object-designator log-node-id :annotation "object-acted-on")

    ;; grab the source object
    (plan-library::grab-object-from-location
     arm source-object-designator (tf-utilities::designator-property-as-location-designator
                                   source-object-designator :object-initial-location-frame) grasping-designator)

    ;; lift the source object over its initial location
    (let ((source-object-designator-projected
            (tf-utilities::start-projecting-object-at-location
             source-object-designator (tf-utilities::designator-property-as-location-designator
                                       source-object-designator :object-over-initial-location-frame))))
      
      (plan-library:singular-arm-move-at-location
       arm (tf-utilities::designator-property-as-location-designator
            (designator-library::particularise-action-designator-for-object-designator
             grasping-designator source-object-designator-projected) :grasping-position))

      (roslisp:ros-info (pour-from-object-into-object) "moved source over it's initial location ..."))

    ;; look at destination object
    (plan-library:sighted
     (tf-utilities::designator-property-as-location-designator destination-object-designator :object-body-frame))
    (roslisp:ros-info (pour-from-object-into-object) "looked at destination object ...")

    ;; recognize the destination object
    (plan-library:visualized destination-object-designator)
    (roslisp:ros-info (pour-from-object-into-object) "recognized the destination object ...")

    ;; move the source object next to the destination object
    (let ((source-object-designator-projected
            (tf-utilities::start-projecting-object-at-location
             source-object-designator (tf-utilities::designator-property-as-location-designator
                                       destination-object-designator :object-poured-0-frame))))
      (plan-library:singular-arm-move-at-location
       arm (tf-utilities::designator-property-as-location-designator
            (designator-library::particularise-action-designator-for-object-designator
             grasping-designator source-object-designator-projected) :grasping-position)))

    ;; move the source object trought the pouring positions
    (dolist (tilting-position
             '(:object-poured-0-frame :object-poured-1-frame :object-poured-2-frame :object-poured-3-frame
               :object-poured-2-frame :object-poured-1-frame))
      
      ;; tilt the source object into over the destination object
      (let ((source-object-designator-projected
              (tf-utilities::start-projecting-object-at-location
               source-object-designator (tf-utilities::designator-property-as-location-designator
                                         destination-object-designator tilting-position))))
        (plan-library:singular-arm-move-at-location
         arm (tf-utilities::designator-property-as-location-designator
              (designator-library::particularise-action-designator-for-object-designator
               grasping-designator source-object-designator-projected) :grasping-position)))

      (roslisp:ros-info (pour-from-object-into-object) "tilted bottle into ~A location ..." tilting-position))

    ;; look at initial location of the source object
    (plan-library:sighted
     (tf-utilities::designator-property-as-location-designator source-object-designator :object-over-initial-location-frame))

    ;; move the source oject over its initial location
    (let ((source-object-designator-projected
            (tf-utilities::start-projecting-object-at-location           
             source-object-designator (tf-utilities::designator-property-as-location-designator
                                       source-object-designator :object-over-initial-location-frame))))
      
      (plan-library:singular-arm-move-at-location
       arm (tf-utilities::designator-property-as-location-designator
            (designator-library::particularise-action-designator-for-object-designator
             grasping-designator source-object-designator-projected) :grasping-position))

      (roslisp:ros-info (pour-from-object-into-object) "moved source over it's initial location ..."))

    ;; place the source into its initial locaion
    (plan-library::place-object-at-location
     arm source-object-designator (tf-utilities::designator-property-as-location-designator
                                   source-object-designator :object-initial-location-frame) grasping-designator)

    (beliefstate:stop-node log-node-id :success T)))

;;; ======================================================================= ;;;
;;; press-object-part (object-designator object-part-designator goal-state) ;;;
;;; ======================================================================= ;;;

(defun press-object-part (arm object-designator object-part-designator goal-state)
  
  ;; todo(lisca): refactor this plan into a cleaner one
  ;;              this plan could be used for flipping a switch and for pressing pipette's button

  (let ((log-node-id (beliefstate:start-node "OPERATING-ELECTRICAL-DEVICE" nil)))

    (beliefstate:add-designator-to-node object-designator log-node-id :annotation "object-acted-on")

    (roslisp:ros-info (press-object-part) "pressed object's part ...")

    ;; move the gripper in front of object part to be pressed
    (plan-library::singular-arm-move-at-location
     arm (tf-utilities::designator-property-as-location-designator object-part-designator :object-front-frame))

    ;; close the gripper
    (plan-library:singular-gripper-gripped arm 0.000 40.000)

    ;; slide from off state towards on state
    (if (eq goal-state :on-state)
        (progn
          (plan-library:singular-arm-move-at-location
           arm (tf-utilities::designator-property-as-location-designator object-part-designator :object-off-state-frame))
          (plan-library:singular-arm-move-at-location
           arm (tf-utilities::designator-property-as-location-designator object-part-designator :object-on-state-frame))
          (roslisp:ros-info 'press-object-part "switched on ...")))

    ;; slide from on state towards off state
    (if (eq goal-state :off-state)
        (progn        
          (plan-library:singular-arm-move-at-location
           arm (tf-utilities::designator-property-as-location-designator object-part-designator :object-on-state-frame))
          (plan-library:singular-arm-move-at-location
           arm (tf-utilities::designator-property-as-location-designator object-part-designator :object-off-state-frame))
          (roslisp:ros-info 'press-object-part "switched off ...")))

    ;; move the gripper in front of the part to be pressed
    (plan-library:singular-arm-move-at-location
     arm (tf-utilities::designator-property-as-location-designator object-part-designator :object-front-frame))

    (roslisp:ros-info (press-object-part) "released object's part ...")

    (beliefstate:stop-node log-node-id :success T)))


;;;
;;; dual arm manipulations
;;;

;;; ============================================================================================ ;;;
;;; dual-arm-move-object-at-location (object-designator location-designator grasping-designator) ;;;
;;; ============================================================================================ ;;;

(defun dual-arm-move-object-at-location (object-designator location-designator grasping-designator)
  ;; note(lisca): this will not work for moving pipette's tip inside
  ;;              container while keeping pushed it's button.

  ;; annotate the motion of the object at a specific location
  (let ((log-node-id (beliefstate:start-node "DUAL-ARM-MOVE-AT-LOCATION" nil)))

    (beliefstate::annotate-resource
     "objectActedOn" (cram-designators:desig-prop-value object-designator :name) "knowrob")

    (roslisp:ros-info (dual-arm-move-object-at-location) "start moving the object into destination location ...")
    
    ;; sight towards object destination location
    (plan-library:sighted location-designator)
    (roslisp:ros-info (dual-arm-move-object-at-location) "sighted towards object destination location ...")
    
    (let ((object-designator-projected
            (tf-utilities::start-projecting-object-at-location object-designator location-designator)))
      
      ;; move the object into the specified location.
      (plan-library::dual-arm-move-at-locations
       (tf-utilities::designator-property-as-location-designator
        (designator-library::particularise-action-designator-for-object-designator
         grasping-designator object-designator-projected) :left-grasping-position)
       (tf-utilities::designator-property-as-location-designator
        (designator-library::particularise-action-designator-for-object-designator
         grasping-designator object-designator-projected) :right-grasping-position)))

    (roslisp:ros-info (move-object-at-location) "finished moving the object into destination location ...")

    (beliefstate:stop-node log-node-id :success T)))
