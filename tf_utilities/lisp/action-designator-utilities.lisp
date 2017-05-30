;;; Copyright (c) 2013, 2016 Gheorghe Lisca <lisca@cs.uni-bremen.de>
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

(in-package :tf-utilities)

;;;;TODO(lisca): 1. What's the purpose of this function???
;;;;             2. Replace this function by a more general one.
;;;;             3. Move it into a more general package dealing with designators
(defun create-singular-gripper-gripping-designator
    (&key gripper gripper-opening-amount gripper-effort)
  (make-instance 'cram-designators:designator
    :description `((:to :set-gripper-opening-amount)
                   (:gripper ,gripper)
                   (:gripper-opening-amount ,gripper-opening-amount)
                   (:gripper-effort ,gripper-effort))))

;;;;TODO(lisca): 1. What's the purpose of this function???
;;;;             2. Replace this function by a more general one.
;;;;             3. Move it into a more general package dealing with designators
(defun create-dual-arm-amount-gripping-designator
    (&key left-gripper-opening-amount right-gripper-opening-amount
       left-gripper-effort right-gripper-effort)
  (make-instance 'cram-designators:designator
    :description `((:to :set-both-grippers-opening-amount)
                   (:left-griper-opening-amount ,left-gripper-opening-amount)
                   (:right-griper-opening-amount ,right-gripper-opening-amount)
                   (:left-griper-effort ,left-gripper-effort)
                   (:right-griper-effort ,right-gripper-effort))))

;;;;TODO(lisca): 1. What's the purpose of this function???
(defun create-singular-arm-grasping-ungrasping-designator
    (arm action pre-grasping-location-designator grasping-location-designator)
  "TODO (lisca): TBD"

  (assert arm)
  (assert action)
  (assert pre-grasping-location-designator)
  (assert grasping-location-designator)

  (let ((pre-grasping-frame (cram-designators:desig-prop-value pre-grasping-location-designator :location-frame))
        (grasping-frame (cram-designators:desig-prop-value grasping-location-designator :location-frame)))

    (assert pre-grasping-frame)
    (assert grasping-frame)

    (make-instance 'cram-designators:designator
      :description `((:to ,action)
                     (:arm ,arm)
                     (:object-pre-grasping-frame ,pre-grasping-frame)
                     (:object-grasping-frame ,grasping-frame)))))

;;;;TODO(lisca): 1. What's the purpose of this function???
(defun create-dual-arm-grasping-ungrasping-designator
    (action object-designator)

  (assert action)
  (assert object-designator)

  (let ((left-pre-grasping-frame (cram-designators:desig-prop-value object-designator :object-left-pre-grasping-frame))
        (right-pre-grasping-frame (cram-designators:desig-prop-value object-designator :object-right-pre-grasping-frame))
        (left-grasping-frame (cram-designators:desig-prop-value object-designator :object-left-grasping-frame))
        (right-grasping-frame (cram-designators:desig-prop-value object-designator :object-right-grasping-frame)))

    (assert left-pre-grasping-frame)
    (assert right-pre-grasping-frame)
    (assert left-grasping-frame)
    (assert right-grasping-frame)

    (make-instance 'cram-designators:designator
      :description `((:to :dual-arm-grasp)
                     (:object-left-pre-grasping-frame ,left-pre-grasping-frame)
                     (:object-right-pre-grasping-frame ,right-pre-grasping-frame)
                     (:object-left-grasping-frame ,left-grasping-frame)
                     (:object-right-grasping-frame ,right-grasping-frame)))))


;;;;TODO(lisca): 1. What's the purpose of this function???
;;;;             2. Replace this function by a more general one.
(defun create-singular-arm-moving-at-location-designator (arm location-designator)

  (assert arm)
  (assert location-designator)

  (let ((grasping-frame (cram-designators:desig-prop-value location-designator :location-frame)))

    (assert grasping-frame)

    (make-instance 'cram-designators:designator
      :description `((:to :singular-arm-move-at-location)
                     (:arm ,arm)
                     (:object-grasping-frame ,grasping-frame)))))

;;;;TODO(lisca): 1. What's the purpose of this function???
;;;;             2. Replace this function by a more general one.
(defun create-dual-arm-moving-at-locations-designator
    (left-hand-location-designator right-hand-location-designator)

  (assert left-hand-location-designator)
  (assert right-hand-location-designator)

  (let ((location-left-frame (cram-designators:desig-prop-value left-hand-location-designator :location-frame))
        (location-right-frame (cram-designators:desig-prop-value right-hand-location-designator :location-frame)))

    (assert location-left-frame)
    (assert location-right-frame)

    (make-instance 'cram-designators:designator
      :description `((:to :dual-arm-move-at-location)
                     (:object-left-grasping-frame ,location-left-frame)
                     (:object-right-grasping-frame ,location-right-frame)))))
