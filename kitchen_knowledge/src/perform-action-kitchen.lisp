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
 

(def-fact-group popcorn-actions ()

  ;;; popcorn 
  (<- (cooking-actions popcorn
                       ?container
                       ?ingredients 
                       ?cooking-location
                       ?serving-location
                          ((cover ?container)
                           (put ?container ?cooking-location)
                           (turn-on ?cooking-location)
                           (wait popcorn)
                           (turn-off ?cooking-location)
                           (put ?container ?serving-location)
                           (uncover ?container)
                           )))

  (<- (serving-actions popcorn 
                       ?serving-container
                       ?serving-ingredients
                       ?serving-location
                       (
                       (pouring popcorn ?serving-container)
                       (salting popcorn)
                       )))
   

  (<- (cooking-steps (steps for cooking ?A) ?Steps)
      (cooking-actions ?A ?Steps))
  
  (<- (container-cover pot lid))
  (<- (container-uncover pot lid))
  (<- (object-location pot table))
  (<- (object-location lid right-drawer))
  (<- (object-location corn right-drawer))
  (<- (object-location oil cupboard))
  (<- (cooking-location popcorn stove))
  (<- (serving-location popcorn table))
  (<- (time popcorn four))

  (<- (serving-steps (steps for serving ?A) ?Steps)
      (serving-actions ?A ?Steps))
 
  (<- (serving-object-location deep-plate table))
  (<- (serving-object-location salt table))
  (<- (serving-object-location popcorn in-pot)) 


;;; pizza
 (<- (cooking-actions pizza
                      ?container
                      ?ingredients 
                      ?cooking-location
                      ?serving-location
                          (
                           (put ?container ?cooking-location)
                           (turn-on ?cooking-location)
                           (wait pizza)
                           (turn-off ?cooking-location)
                           (put ?container ?serving-location)
                           )))
  
(<- (serving-actions pizza 
                     ?serving-container
                     ?serv-ingredients
                     ?serving-location
                       (
                       (slice pizza)
                       (put-ingredients ?serv-ingredients ?serving-location)
                       
                        )))

 (<- (object-location baking-sheet table))
 (<- (object-location dough fridge))
 (<- (object-location tomato-sauce fridge))
 (<- (object-location cheese fridge))
 (<- (cooking-location pizza oven))
 (<- (serving-location pizza table))

 (<- (serving-object-location small-plate table))
 (<- (serving-object-location pizza  on-baking-sheet))
 (<- (serving-object-location pepper cupboard))
 (<- (serving-object-location rucola fridge))
 (<- (time pizza thirty))

  
)
 
 

(defun perform-action (action)
    (cond
    ((eql (car action) 'cover) 
       (cover (second action)))
    ((eql (car action) 'put)
       (put (second action) (third action)))
    ((eql (car action) 'turn-on)
       (turn-on (second action)))
    ((eql (car action) 'wait) 
       (wait (second action)))
    ((eql (car action) 'turn-off)
       (turn-off (second action)))
    ((eql (car action) 'uncover) 
       (uncover (second action)))
    ((eql (car action) 'pouring) 
       (pouring (second action) (third action)))
    ((eql (car action) 'salting)
       (salting (second action) ))
     ((eql (car action) 'slice)
       (slice (second action) ))
     ((eql (car action) 'put-ingredients) 
       (put-ingredients (second action) (third action)))
    (T
      (format t "ERROR: I don't understand ~a~%" action))))


(defun tell-how-to-cook (recipe-name)
  (let*  ((container (car (get-cooking-container (car (get-cooking-ingredients recipe-name))))) 
         ;;((container (car (get-cooking-container recipe-name)))
         (container-location (car (get-object-location container)))
         (ingredients (car (get-cooking-ingredients recipe-name)))
         (ingredient-locations (mapcar (lambda (ingredient)
                                         (car (get-object-location ingredient)))
                                       ingredients))
         (cooking-location (car (get-cooking-location recipe-name)))
         (serving-location (car (get-serving-location recipe-name)))
         (actions (car (get-cooking-steps recipe-name container ingredients cooking-location serving-location))))                                                                    
      


    (format t "  I will use the ~a to cook. (I can find it on the ~a).~%" container container-location)
    (format t "  I will need these ingredients: ~a. Respectively, I can find each in ~a.~%" ingredients ingredient-locations)
    (format t "  I will put the ingredients (in the above order) in the ~a. Then:~%" container)


    (loop for action in actions do
      (perform-action action))))


(defun tell-how-to-serve (recipe-name)
  (let*  ((serving-container (car (get-serving-container (car (get-serving-ingredients recipe-name))))) 
         ;;((serving-container (car (get-serving-container recipe-name)))
         (serv-ingredients (car (get-serving-ingredients recipe-name)))
         (serv-ingredient-location (mapcar (lambda (serv-ingredient)
                                         (car (get-serving-object-location serv-ingredient)))
                                       serv-ingredients))
         (serving-location (car (get-serving-location recipe-name)))
         (actions (car (get-serving-steps recipe-name serving-container serv-ingredients serving-location))))                                                                     
      


    (format t "  I will use the ~a to serve. (I can find it on the ~a).~%" serving-container serving-location)
    (format t "  I will need this ingredient: ~a. Respectively, I can find it on the ~a.~%" serv-ingredients serv-ingredient-location)
    (format t "  Then:~%" )


    (loop for action in actions do
      (perform-action action)))) 
