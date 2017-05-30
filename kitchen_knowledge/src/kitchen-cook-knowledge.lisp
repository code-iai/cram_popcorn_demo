;;; Copyright (c) 2017, Boglarka Erdodi <erdoedib@uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :kitchen-knowledge)
 
(defun get-cooking-steps (subst-A container ingredients cooking-location serving-location)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?Steps bdgs))
	  (force-ll (prolog:prolog `(kitchen-knowledge:cooking-actions ,subst-A ,container ,ingredients ,cooking-location ,serving-location ?Steps)))))

(defun get-container-cover (container)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?cover bdgs))
          (force-ll (prolog:prolog `(cooking:container-cover ,container ?cover)))))

(defun get-container-uncover (container)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?uncover bdgs))
          (force-ll (prolog:prolog `(cooking:container-uncover ,container ?uncover)))))

(defun get-object-location (object)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?location bdgs))
          (force-ll (prolog:prolog `(cooking:object-location ,object ?location)))))

(defun get-cooking-location (recipe-name)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?location bdgs))
          (force-ll (prolog:prolog `(cooking:cooking-location ,recipe-name ?location)))))

(defun get-container-put (container)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?put bdgs))
          (force-ll (prolog:prolog `(cooking:object-location ,container ?put)))))

(defun get-serving-location (recipe-name)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?location bdgs))
          (force-ll (prolog:prolog `(cooking:serving-location ,recipe-name ?location)))))
 
(defun get-time-cook (recipe-name)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?time bdgs))
          (force-ll (prolog:prolog `(cooking:time ,recipe-name ?time)))))
 
(defun cover (container)
  (let* ((cover-with (car (get-container-cover container)))
         (location-for-cover (car (get-object-location cover-with))))
    (format t "  I will cover the ~a with the ~a (I can find the ~a in the ~a).~%" 
             container cover-with cover-with location-for-cover)))

(defun uncover (container)
  (let* ((uncover-the (car (get-container-uncover container))))
    (format t "  I will uncover the ~a from the ~a.~%" 
              uncover-the container)))


(defun put (container location)
  (format t "  I will put the ~a on the ~a. ~%" 
          container location))

 (defun turn-on (turn-object)
   (format t "  I will turn on the ~a. ~%" 
           turn-object))

(defun turn-off (turn-object)
   (format t "  I will turn off the ~a. ~%" 
           turn-object))

(defun wait (recipe-name)
 (let* ((time (car (get-time-cook recipe-name))))
   (format t "  I will wait ~a minutes. ~%" 
           time)))


      
 
