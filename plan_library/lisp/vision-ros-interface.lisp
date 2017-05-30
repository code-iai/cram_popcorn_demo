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

(in-package :plan-library)

(defparameter *robosherlock-service-type*
  'iai_robosherlock_msgs-srv:perceiveobject
  ;; 'iai_robosherlock_msgs-srv:pizzaperceiveobject
  )

(defparameter *robosherlock-service-name*
  "/RoboSherlock_popcorn/perceive_kitchen_objects"
  ;; "/robosherlock_to_cram/perceive"
  )

(defparameter *robosherlock-pipeline-processing-duration* 3.0)

(defparameter *table-surface-height* 0.690)

;; Note(lisca): this is the newest!
;; (defun recognize-object (object-designator)

;;   (assert object-designator)

;;   (let ((requested-object-type (cram-designators:desig-prop-value object-designator :object-type)))

;;     (roslisp:ros-info
;;      (recognize-object) "waiting for roboSherlock's service ~a ... " *robosherlock-service-name*)
    
;;     (roslisp::wait-for-service *robosherlock-service-name*)
    
;;     (roslisp:ros-info
;;      (recognize-object) "asking roboSherlock for ~a ... " requested-object-type)

;;     (let ((object-stamped-transform

;;             ;; keep asking robosherlock until we get an answer
;;             (loop

;;               (let ((robosherlock-object-stamped-transform
                      
;;                       (block robosherlock-querry

;;                         (handler-case

;;                             (roslisp:call-service
;;                              *robosherlock-service-name* *robosherlock-service-type*
;;                              :object_name requested-object-type)

;;                           (roslisp:service-call-error ()
;;                             (progn
;;                               (roslisp:ros-warn
;;                                (recognize-object) "roboSherlock didn't recgonize the ~A ... !!!" requested-object-type)
;;                               (return-from robosherlock-querry NIL)))))))

;;                 (if robosherlock-object-stamped-transform
;;                     (return robosherlock-object-stamped-transform)
;;                     (roslisp:ros-warn (recognize-object) "asking it again in 0.5 second ..."))

;;                 (sleep 0.5)))))
      
;;       ;; ;; Overwrite the height of all objects!
;;       ;; (let ((object-stamped-transform
;;       ;;         robosherlock-object-stamped-transform
;;       ;;         ;; (roslisp:modify-message-copy robosherlock-object-stamped-transform
;;       ;;         ;;                  (z translation transform object_transf) *table-surface-height*)
;;       ;;         )))

;;       (roslisp:with-fields (object_transf) object-stamped-transform
;;         (roslisp:with-fields (header child_frame_id) object_transf
;;           (roslisp:with-fields (frame_id) header
            
;;             (cond ((not (equal child_frame_id frame_id))

;;                    ;; Broadcast the updated transform.
;;                    (cl-transforms-stamped:start-publishing-transforms-static-globaly
;;                     (cl-transforms-stamped:from-msg object_transf))

;;                    ;; ;; Wait for approx 2.0 seconds such
;;                    ;; ;; that the newly broadcasted transform will
;;                    ;; ;; replace the old one.
;;                    (sleep 0.5)

;;                    (roslisp:ros-info (recognize-object) "We've got from RoboSherlock ~a ..." child_frame_id))
                  
;;                   (t (roslisp:ros-info
;;                       (recognize-object) "We've got from RoboSherlock ... NOTHING ... but a weird StampedTransform ...")))))))))

;; (defun recognize-object (object-designator)

;;   (assert object-designator)

;;   (let ((requested-object-type (cram-designators:desig-prop-value object-designator :object-type)))

;;     (roslisp:ros-info
;;      (recognize-object) "waiting for roboSherlock's service ~a ... " *robosherlock-service-name*)
    
;;     (roslisp::wait-for-service *robosherlock-service-name*)
    
;;     (roslisp:ros-info
;;      (recognize-object) "asking roboSherlock for ~a ... " requested-object-type)

;;     (let ((object-stamped-transform

;;             ;; keep asking robosherlock until we get an answer
;;             (loop

;;               (let ((robosherlock-object-stamped-transform
                      
;;                       (block robosherlock-querry

;;                         (handler-case

;;                             (roslisp:call-service
;;                              *robosherlock-service-name* *robosherlock-service-type*
;;                              :object_name requested-object-type
;;                              :apply_pos_rot_fixes t
;;                              :stick_to_map t
;;                              :publish_transform_once nil)

;;                           (roslisp:service-call-error ()
;;                             (progn
;;                               (roslisp:ros-warn
;;                                (recognize-object) "roboSherlock didn't recgonize the ~A ... !!!" requested-object-type)
;;                               (return-from robosherlock-querry NIL)))))))

;;                 (if robosherlock-object-stamped-transform
;;                     (return robosherlock-object-stamped-transform)
;;                     (roslisp:ros-warn (recognize-object) "asking it again in 0.5 second ..."))

;;                 (sleep 0.5)))))
      
;;       ;; ;; Overwrite the height of all objects!
;;       ;; (let ((object-stamped-transform
;;       ;;         robosherlock-object-stamped-transform
;;       ;;         ;; (roslisp:modify-message-copy robosherlock-object-stamped-transform
;;       ;;         ;;                  (z translation transform object_transf) *table-surface-height*)
;;       ;;         )))

;;       (roslisp:with-fields (object_transf) object-stamped-transform
;;         (roslisp:with-fields (header child_frame_id) object_transf
;;           (roslisp:with-fields (frame_id) header
            
;;             (cond ((not (equal child_frame_id frame_id))

;;                    ;; Broadcast the updated transform.
;;                    (cl-transforms-stamped:start-publishing-transforms-static-globaly
;;                     (cl-transforms-stamped:from-msg object_transf))

;;                    ;; ;; Wait for approx 2.0 seconds such
;;                    ;; ;; that the newly broadcasted transform will
;;                    ;; ;; replace the old one.
;;                    (sleep 0.5)

;;                    (roslisp:ros-info (recognize-object) "We've got from RoboSherlock ~a ..." child_frame_id))
                  
;;                   (t (roslisp:ros-info
;;                       (recognize-object) "We've got from RoboSherlock ... NOTHING ... but a weird StampedTransform ...")))))))))

(defun recognize-object (object-designator)

  (assert object-designator)

  (let ((requested-object-type
          (cram-designators:desig-prop-value object-designator :object-type)))

    (roslisp:ros-info
     'vision-recognition-process-module "We wait for  RoboSherlock's service ~a ... " *robosherlock-service-name*)
    
    (roslisp::wait-for-service *robosherlock-service-name*)
    
    (roslisp:ros-info
     'vision-recognition-process-module "We ask RoboSherlock for ~a ... " requested-object-type)

    (let ((object-stamped-transform
            (loop

              ;; Note: Helper call for RoboSherlock
              ;; Mimic the pressing of joystick's left button nr. 2
              ;; - by publishing on /joy topic a message.
              (let ((joystick-topic
                      (roslisp:advertise "/joy" "sensor_msgs/Joy")))
                (roslisp::publish
                 joystick-topic
                 (roslisp::make-message
                  "sensor_msgs/Joy"
                  (axes) (vector 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 -1.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
                  (buttons) (vector 0 0 0 0 0 0 0 0 0 1  0 0 0 0 0 0 0))))

              (roslisp:ros-info
               'recognize-object "We wait ~A seconds for RoboSherlock to reconize the ~A ..."
               *roboSherlock-pipeline-processing-duration*
               requested-object-type)
              
              (sleep *roboSherlock-pipeline-processing-duration*)
              
              (let ((robosherlock-object-stamped-transform
                      
                      (block robosherlock-querry

                        (handler-case

                            (roslisp:call-service
                             "/RoboSherlock_popcorn/perceive_kitchen_objects"                          
                             'iai_robosherlock_msgs-srv:perceiveobject
                             :object_name requested-object-type)

                          (roslisp::simple-error ()
                            (progn
                              (roslisp:ros-warn
                               'vision-recognition-process-module
                               "RoboSherlock didn't recognize the ~A ... !!!" requested-object-type)
                              (return-from robosherlock-querry NIL)))
                          
                          (roslisp::service-error ()
                            (progn
                              (roslisp:ros-warn
                               'vision-recognition-process-module
                               "RoboSherlock didn't recognize the ~A ... !!!" requested-object-type)
                              (return-from robosherlock-querry NIL)))
                          
                          (roslisp:service-call-error ()
                            (progn
                              (roslisp:ros-warn
                               'vision-recognition-process-module
                               "RoboSherlock didn't recgonize the ~A ... !!!" requested-object-type)
                              (return-from robosherlock-querry NIL)))))))

                (if robosherlock-object-stamped-transform
                    (return robosherlock-object-stamped-transform)
                    (roslisp:ros-warn
                     'vision-recognition-process-module
                     "Asking it again in 1.0 second ..."))

                (sleep 0.5))))
          )
      
      ;; ;; Overwrite the height of all objects!
      ;; (let ((object-stamped-transform
      ;;         robosherlock-object-stamped-transform
      ;;         ;; (roslisp:modify-message-copy robosherlock-object-stamped-transform
      ;;         ;;                  (z translation transform object_transf) *table-surface-height*)
      ;;         )))

      (roslisp:with-fields
          (object_transf)
          object-stamped-transform
        (roslisp:with-fields
            (header child_frame_id)
            object_transf
          (roslisp:with-fields
              (frame_id)
              header
            (cond ((not (equal child_frame_id frame_id))
                   ;; Broadcast the updated transform.
                   (cl-transforms-stamped:start-publishing-transforms-static-globaly
                    (cl-transforms-stamped:from-msg object_transf))

                   ;; ;; Wait for approx 2.0 seconds such
                   ;; ;; that the newly broadcasted transform will
                   ;; ;; replace the old one.
                   ;; (sleep 2.0)

                   (roslisp:ros-info 'vision-recognition-process-module "We've got from RoboSherlock ~a ..." child_frame_id))
                  
                  (t (roslisp:ros-info
                      'vision-recognition-process-module
                      "We've got from RoboSherlock ... NOTHING ... but a weird StampedTransform ...")))))))))
