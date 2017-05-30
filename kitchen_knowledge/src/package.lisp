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

(in-package :cl-user)

(defpackage kitchen-knowledge
  (:nicknames :cooking)
  (:use #:common-lisp
        #:cram-prolog
        #:cram-utilities)
  (:export get-cooking-container
           cooking-container
           get-serving-container
           serving-container
           cooking-ingredients
           get-cooking-ingredients
           serving-ingredients
           get-serving-ingredients
           cooking-steps
           get-cooking-steps
           serving-steps
           get-serving-steps
           objects-actions
           cooking-actions
           serving-actions
           popcorn-actions
           container-cover
           container-uncover
           container-put
           object-location
           get-container-cover
           get-container-uncover
           get-object-location
           get-cooking-location
           get-container-put
           serving-location
           get-serving-location
           wait
           time
           get-time-cook
           tell-how-to-cook
           tell-how-to-serve
           get-serving-object-location
           serving-object-location
           pouring
           salting
           turn-on
           cooking-location
           cover
           put
           corn
           oil
           popcorn
           pot
           small-bowl
           deep-plate
           salt
           
           pizza
           baking-sheet
           small-plate
           dough
           tomato-sauce
           cheese
           pepper
           rucola
           slice
           put-ingredients
           serv-ingredients
           serving-location
           ))
