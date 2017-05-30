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

(defparameter *perceive-action-name*
  "perceive_action")

(defparameter *perceive-action-type*
  "popcorn_perception/PerceiveAction")

(defparameter *perceive-action-timeout* 30.0
  "How many seconds to wait before returning from the perception action.")

(defvar *perceive-action-client* nil)


(defun init-perceive-action-client ()
  (setf *perceive-action-client* (actionlib:make-action-client
                                  *perceive-action-name*
                                  *perceive-action-type*))
  (loop until (actionlib:wait-for-server *perceive-action-client* 30.0)
        do (roslisp:ros-info (perceive-action-client) "Waiting for Perceive action server..."))
  (roslisp:ros-info (perceive-action-client) "Perceive action client created.")
  *perceive-action-client*)

(defun destroy-perceive-action-client ()
  (setf *perceive-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-perceive-action-client)

(defun get-perceive-action-client ()
  (or *perceive-action-client*
      (init-perceive-action-client)))

(defun make-perceive-action-goal (object-name-string object-frame-id-string)
  (actionlib:make-action-goal
      (get-perceive-action-client)
    object_type object-name-string
    object_frame_id_name object-frame-id-string))

(defun ensure-perceive-goal-reached (status)
  (when (eql status :timeout)
    (cpl:fail 'actionlib-action-timed-out :description "Perceive action timed out."))
  ;;  (unless (eql status :succeeded)
  ;;  (cpl:fail 'pr2-low-level-failure :description "Perceive action did not succeed."))
  )


(defun call-perceive-action (&key (object-name "nothing")
                               (object-frame-id-name "map")
                               (action-timeout *perceive-action-timeout*))
  (loop
    (multiple-value-bind (result status)
        (cpl:with-failure-handling
            ((simple-error (e)
               (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
               (init-perceive-action-client)
               (cpl:retry)))
          
          (let ((actionlib:*action-server-timeout* *perceive-action-timeout*))
            (actionlib:call-goal
             (get-perceive-action-client)
             (make-perceive-action-goal object-name object-frame-id-name)
             :timeout action-timeout)))
      
      (ensure-perceive-goal-reached status)
     

      (if (eql status :succeeded)
          (return  (values result status))
          (progn
            (format t "Object ~A NOT found... Retrying in 0.5 sec.~%" object-name)
            (sleep 0.5))))))
