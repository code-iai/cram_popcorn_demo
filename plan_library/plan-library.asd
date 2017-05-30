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

(defsystem plan-library
  :author "Gheorghe Lisca"
  :license "BSD"

  :depends-on (sensor_msgs-msg
               move_base_msgs-msg
               geometry_msgs-msg
               pr2_controllers_msgs-msg
               two_hand_ik_trajectory_executor-srv
               iai_robosherlock_msgs-srv
               giskard_msgs-msg
               giskard_msgs-srv
               iai_wsg_50_msgs-msg
               ;; boxy_moveit_config-msg
               
               tf-utilities
               roslisp-utilities
               actionlib
               cl-tf2
               cram-utilities
               cram-language
               cram-beliefstate

               designator-library)

  :components
  ((:module "lisp"
    :components

    ((:file "package-interface")

     (:file "navigation-ros-interface"
      :depends-on ("package-interface"))

     (:file "navigation-plans"
      :depends-on ("package-interface"
                   "navigation-ros-interface"))

     (:file "torso-ros-interface"
      :depends-on ("package-interface"))

     (:file "torso-plans"
      :depends-on ("package-interface"
                   "torso-ros-interface"))
     
     (:file "sighting-ros-interface"
      :depends-on ("package-interface"))
     
     (:file "sighting-plans"
      :depends-on ("package-interface"
                   "sighting-ros-interface"))

     (:file "vision-ros-interface"
      :depends-on ("package-interface"))
     
     (:file "vision-plans"
      :depends-on ("package-interface"
                   "vision-ros-interface"))

      (:file "perception-ros-interface"
      :depends-on ("package-interface"))
     
     (:file "perception-plans"
      :depends-on ("package-interface"
                   "perception-ros-interface"))

     (:file "inverse-kinematics-ros-interface"
      :depends-on ("package-interface"))
     
     (:file "constraint-controller-ros-interface"
      :depends-on ("package-interface"))
     
     (:file "singular-arm-manipulation-plans"
      :depends-on ("package-interface"
                   "inverse-kinematics-ros-interface"
                   "constraint-controller-ros-interface"))
     
     (:file "dual-arm-manipulation-plans"
      :depends-on ("package-interface"
                   "inverse-kinematics-ros-interface"
                   "constraint-controller-ros-interface"))

     (:file "manipulation-plans"
      :depends-on ("package-interface"
                   "singular-arm-manipulation-plans"
                   "dual-arm-manipulation-plans"))
     

     (:file "singular-gripper-gripping-ros-interface"
      :depends-on ("package-interface"))

     (:file "singular-gripper-gripping-plans"
      :depends-on ("package-interface"
                   "singular-gripper-gripping-ros-interface"))

     (:file "dual-gripper-gripping-ros-interface"
      :depends-on ("package-interface"))
     
     (:file "dual-gripper-gripping-plans"
      :depends-on ("package-interface"
                   "dual-gripper-gripping-ros-interface"))

     ;;; Added new plans for the popcorn-demo 
     
     (:file "grasp-object-single-arm-plans"
      :depends-on ("package-interface"))

     (:file "grasp-object-dual-arm-plans"
      :depends-on ("package-interface"))

     (:file "ungrasp-object-single-arm-plans"
      :depends-on ("package-interface"))

     (:file "ungrasp-object-dual-arm-plans"
      :depends-on ("package-interface"))
     

     (:file "move-object-plans"
      :depends-on ("package-interface"
                   "singular-arm-manipulation-plans"
                   "dual-arm-manipulation-plans"))))))
