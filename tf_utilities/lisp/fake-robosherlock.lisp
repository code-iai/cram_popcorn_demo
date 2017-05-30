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

(in-package :tf-utilities)

;; The fake perception stuff.

(defun lookup-object-frame (frame-name)

  (roslisp:ros-info
   (tf-utilities) "Asking TF2 about the \"~A\" object frame" frame-name)

  (loop

    (let ((transform-stamped

            (block tf-querry

              (handler-case

                (cl-tf2:lookup-transform
                  *cl-tf2-buffer-client*
                  "map"
                  frame-name
                  :time (roslisp:ros-time)
                  :timeout *cl-tf2-buffer-client-timeout*)

                (cl-transforms-stamped:transform-stamped-error ()
                  (progn
                    (roslisp:ros-warn (tf-utilities) "cl-transforms-stamped:transform-stamped-error !!!")
                    (return-from tf-querry NIL)))

                (cl-transforms-stamped:lookup-error ()
                  (progn
                    (roslisp:ros-warn (tf-utilities) "cl-transforms-stamped:lookup-error !!!")
                    (return-from tf-querry NIL)))))))

      (if transform-stamped
          (return transform-stamped)
          (roslisp:ros-warn
           (tf-utilities) "no transform ~A -> ~A found in the last ~A seconds ... retrying in 0.1 seconds ..."
                             "map" frame-name *cl-tf2-buffer-client-timeout*))

      (sleep 0.1))))

(defun start-fake-robosherlock ()
  "The fake vision looks up the frame of the object trough cl-tf2."
  
  (roslisp:def-service-callback iai_robosherlock_msgs-srv:pizzaperceiveobject (object_name)

    ;;TODO(lisca): lookup the requested frames and return them!
    (roslisp:make-response
     :object_transf (cl-transforms-stamped:to-msg
                     (cond ((equal object_name "tool_holder")
                             (lookup-object-frame "tool_holder_frame"))
                           
                           ((equal object_name "tool_holder_rack_0")
                             (lookup-object-frame "tool_holder_rack_0_frame"))

                            ((equal object_name "tool_holder_rack_1")
                             (lookup-object-frame "tool_holder_rack_1_frame"))

                            
                            ((equal object_name "spoon_rounded")
                             (lookup-object-frame "spoon_rounded_frame"))

                            ((equal object_name "spoon_flatten")
                             (lookup-object-frame "spoon_flatten_frame"))

                            ((equal object_name "red_bowl")
                             (lookup-object-frame "red_bowl_frame"))

                            ((equal object_name "yellow_bowl")
                             (lookup-object-frame "yellow_bowl_frame"))

                            ((equal object_name "tray")
                             (lookup-object-frame "tray_frame"))

                            ((equal object_name "tomato_sauce")
                             (lookup-object-frame "tomato_sauce_frame"))

                            ((equal object_name "cheese")
                             (lookup-object-frame "cheese_frame"))
                            
                            ((equal object_name "pizza")
                             (lookup-object-frame "pizza_frame"))

                            (t (roslisp:ros-info
                                (tf-utilities) "The object you requested is not detectable ..."))))))

  (roslisp:register-service "/robosherlock_to_cram/perceive" 'iai_robosherlock_msgs-srv:pizzaperceiveobject)

  (roslisp:ros-info (tf-utilities) "ready to provide fake robosherlock vision ..."))
