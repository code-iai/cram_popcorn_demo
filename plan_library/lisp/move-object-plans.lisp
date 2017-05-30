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
;;;

(in-package :plan-library)


;;; ==================================================================================== ;;;
;;; single-arm-move-object-at-location (arm object-designator final-location-designator) ;;;
;;; ==================================================================================== ;;;


(defun single-arm-move-object-at-location
    (arm object-designator final-location-designator
     &optional 
       1st-intermediate-location-designator 2nd-intermediate-location-designator
       3rd-intermediate-location-designator 4th-intermediate-location-designator)

  ;;verify if there are intermediate locations and move the object there
  (loop for intermediate-location-designator in (list  1st-intermediate-location-designator
                                                       2nd-intermediate-location-designator
                                                       3rd-intermediate-location-designator
                                                       4th-intermediate-location-designator)

        when (not (eql intermediate-location-designator NIL))
          do
             ;;project tht object's frames at the next location
             (roslisp:ros-info (single-arm-move-object-at-location) "Projecting the object to the next location...")
             (let ((object-designator-projected
                     (tf-utilities::start-projecting-object-at-location object-designator intermediate-location-designator)))

               ;; move the object into the specified location.
               (roslisp:ros-info (single-arm-move-object-at-location) "Moving the object into location ...")
               (plan-library:singular-arm-move-at-location
                arm (tf-utilities::designator-property-as-location-designator object-designator-projected :object-grasping-frame))))
  
  ;;move to the final location
  ;;project tht object's frames at the final location
  (roslisp:ros-info (single-arm-move-object-at-location) "Projecting the object to the final location...")
  (let ((object-designator-projected
          (tf-utilities::start-projecting-object-at-location object-designator final-location-designator)))

    ;; move the object into the specified location.
    (roslisp:ros-info (single-arm-move-object-at-location) "Moving the object into final location ...")
    (plan-library:singular-arm-move-at-location
     arm (tf-utilities::designator-property-as-location-designator object-designator-projected :object-grasping-frame))

    (tf-utilities::stop-projecting-object-at-location object-designator-projected)))



;;; ==================================================================================== ;;;
;;; double-arm-move-object-at-location (arm object-designator final-location-designator) ;;;
;;; ==================================================================================== ;;;


(defun double-arm-move-object-at-location
    (object-designator final-location-designator
     &optional 
       1st-intermediate-location-designator 2nd-intermediate-location-designator
       3rd-intermediate-location-designator 4th-intermediate-location-designator)

  ;;verify if there are intermediate locations and move the object there
  (loop for intermediate-location-designator in (list  1st-intermediate-location-designator
                                                       2nd-intermediate-location-designator
                                                       3rd-intermediate-location-designator
                                                       4th-intermediate-location-designator)

        when (not (eql intermediate-location-designator NIL))
          do
             ;;project tht object's frames at the next location
             (roslisp:ros-info (single-arm-move-object-at-location) "Projecting the object to the next location...")
             (let ((object-designator-projected
                     (tf-utilities::start-projecting-object-at-location object-designator intermediate-location-designator)))

               (plan-library::dual-arm-move-at-locations
                (tf-utilities::designator-property-as-location-designator object-designator-projected :object-grasping-left-frame)
                (tf-utilities::designator-property-as-location-designator object-designator-projected :object-grasping-right-frame))))

  ;;move object to the final location
  (roslisp:ros-info (single-arm-move-object-at-location) "Projecting the object to final location...")
  (let ((object-designator-projected
          (tf-utilities::start-projecting-object-at-location object-designator final-location-designator)))

    (plan-library::dual-arm-move-at-locations
     (tf-utilities::designator-property-as-location-designator object-designator-projected :object-grasping-left-frame)
     (tf-utilities::designator-property-as-location-designator object-designator-projected :object-grasping-right-frame))

    (tf-utilities::stop-projecting-object-at-location object-designator-projected)))
