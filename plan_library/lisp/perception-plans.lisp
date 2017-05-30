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



(defun detect-object (object-designator)  

  ;; sight toward the object location
  (sighted (tf-utilities::designator-property-as-location-designator
                          object-designator :object-body-frame))
  
  ;; call the perceive action server    
  (multiple-value-bind (result status) (call-perceive-action
                                        :object-name
                                        (cram-designators:desig-prop-value object-designator :object-type)
                                        :object-frame-id-name
                                        (cl-tf:frame-id (cram-designators:desig-prop-value object-designator :object-body-frame)))
    
    (declare (ignore status))

    ;; parse the object position values from the result
    (roslisp:with-fields ((object-pose object_pose)) result  
      (let ((pose-x (aref object-pose 0))
            (pose-y (aref object-pose 1)))

        ;; take the object body frame from the object designator
        (let ((object-new-frame (cram-designators:desig-prop-value object-designator :object-body-frame)))
          
          ;; update the object's translation
          (setf
           (slot-value object-new-frame 'cl-transforms-stamped:translation)
           (cl-transforms:make-3d-vector
            pose-x
            pose-y
            0))

          ;; publish the object's new frame
          (cl-transforms-stamped:start-publishing-transforms-static-globaly object-new-frame)

          ;; wait for tf server to pick up the new transforms
          (sleep 0.5)
          (roslisp:ros-info 'detect-object "Object ~A detected! :)"
                            (cram-designators:desig-prop-value object-designator :object-type)))))))


;;  (first (coerce pv 'list))
;;  (aref pv 0)
;;  (cl-tf:x (cl-tf:translation (cram-designators:desig-prop-value object-designator :object-initial-location-frame)))
