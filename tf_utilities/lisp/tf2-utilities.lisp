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

(in-package tf-utilities)

(defun cl-transform->geometry_msgs-pose (transform)
  (let ((translation
          (cl-transforms:translation transform))
        (rotation
          (cl-transforms:rotation transform)))

    (roslisp:make-message
     "geometry_msgs/Pose"
     (x position) (cl-transforms:x translation)
     (y position) (cl-transforms:y translation)
     (z position) (cl-transforms:z translation)

     (x orientation) (cl-transforms:x rotation)
     (y orientation) (cl-transforms:y rotation)
     (z orientation) (cl-transforms:z rotation)
     (w orientation) (cl-transforms:w rotation))))

(defun transform-stamped->pose-stamped (transform-stamped)
  (make-instance
      'cl-transforms-stamped:pose-stamped
    :frame-id (cl-transforms-stamped:frame-id transform-stamped)
    :stamp (roslisp:ros-time)
    :origin (cl-transforms-stamped:translation transform-stamped)
    :orientation (cl-transforms-stamped:rotation transform-stamped)))

(defparameter *cl-tf2-buffer-client* nil)

(defparameter *cl-tf2-buffer-client-timeout* 1.0)

(defun create-cl-tf2-buffer-client ()
  (setf *cl-tf2-buffer-client*
    (make-instance 'cl-tf2:buffer-client)))

(roslisp-utilities:register-ros-init-function
 create-cl-tf2-buffer-client)

(defun lookup-current-frame-relative-to-refference-frame (current-frame-name reference-frame-name)

  (loop

    (let ((transform

            (block tf-querry

              (handler-case

                (cl-tf2:lookup-transform
                  *cl-tf2-buffer-client*
                  reference-frame-name
                  current-frame-name
                  :time (roslisp:ros-time)
                  :timeout *cl-tf2-buffer-client-timeout*)

                (cl-transforms-stamped:transform-stamped-error ()
                  (progn
                    (roslisp:ros-warn (tf-utilities) "")
                    (roslisp:ros-warn (tf-utilities) "cl-transforms-stamped:transform-stamped-error ... !")
                    (return-from tf-querry NIL)))

                (cl-transforms-stamped:lookup-error ()
                  (progn
                    (roslisp:ros-warn (tf-utilities) "")
                    (roslisp:ros-warn (tf-utilities) "cl-transforms-stamped:lookup-error ... !")
                    (return-from tf-querry NIL)))

                (simple-error ()
                  (progn
                    (roslisp:ros-warn (tf-utilities) "")
                    (roslisp:ros-warn (tf-utilities) "simple-error ... !")
                    (return-from tf-querry NIL)))))))

      (if transform
          (progn
            (roslisp:ros-info (tf-utilities) "")
            (roslisp:ros-info (tf-utilities) "~A -> ~A found ... !" current-frame-name reference-frame-name)
            (roslisp:ros-info (tf-utilities) "")
            (return transform))
          (progn
            (roslisp:ros-warn (tf-utilities) "no transform found in the last ~A seconds ... !" *cl-tf2-buffer-client-timeout*)
            (roslisp:ros-warn (tf-utilities) "~A -> ~A" current-frame-name reference-frame-name)
            (roslisp:ros-warn (tf-utilities) "retrying in 0.1 seconds ...")))

      (sleep 0.1))))
