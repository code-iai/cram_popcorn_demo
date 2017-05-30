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


(def-fact-group objects-actions ()

  ;;; popcorn
  (<- (cooking-containers popcorn (pot)))

  (<- (serving-containers popcorn (deep-plate))) 
  
  (<- (cooking-ingredients popcorn (oil
                                    corn))) 

  (<- (serving-ingredients popcorn (popcorn salt)))

  ;;; pizza
  (<- (cooking-containers pizza (baking-sheet)))

  (<- (serving-containers pizza (small-plate)))

  (<- (cooking-ingredients pizza (dough tomato-sauce
                                    cheese)))

  (<- (serving-ingredients pizza (pizza pepper rucola)))
  
  
  
  ;;;popcorn
  (<- (can-hold deep-plate oil))
  (<- (can-hold deep-plate salt))
  (<- (can-hold deep-plate corn))
  (<- (can-hold deep-plate popcorn))
  (<- (can-hold small-bowl oil))
  (<- (can-hold small-bowl corn))
  (<- (can-hold small-bowl popcorn))
  (<- (can-hold pot oil))
  (<- (can-hold pot corn))
  (<- (can-hold pot popcorn))
  (<- (temperature-resistant pot))
  (<- (small-dish small-bowl))
  (<- (serving-dish deep-plate))

  ;;;pizza
  (<- (can-hold baking-sheet tomato-sauce))
  (<- (can-hold baking-sheet cheese))
  (<- (can-hold baking-sheet dough))
  (<- (can-hold small-plate pizza))
  (<- (can-hold small-plate pepper))
  (<- (can-hold small-plate rucola))
  (<- (temperature-resistant baking-sheet))
  (<- (serving-dish small-plate))
  


  
(<- (cooking-container ?Ingredient-List ?Container)
    (temperature-resistant ?Container)
    (forall (member ?Ingredient ?Ingredient-List) 
            (can-hold ?Container ?Ingredient)))

(<- (serving-container ?Ingredient-List ?Container)
    (serving-dish ?Container)
    (forall (member ?Ingredient ?Ingredient-List) 
            (can-hold ?Container ?Ingredient)))

(<- (cooking-ingredients (cook ?A from) ?Ingredient)
    (cooking-ingredients ?A ?Ingredient))

(<- (serving-ingredients (serv ?A with) ?Ingredient)
    (serving-ingredients ?A ?Ingredient)) 
  ) 




(defun get-cooking-container (ingredient-list)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?C bdgs))
          ;;(alexandria:curry #'cut:substitute-vars '?C)
	  (force-ll (prolog:prolog `(kitchen-knowledge:cooking-container ,ingredient-list ?C)))))

(defun get-serving-container (ingredient-list)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?C bdgs))
	  (force-ll (prolog:prolog `(kitchen-knowledge:serving-container ,ingredient-list ?C)))))

(defun get-cooking-ingredients (subst-A)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?C bdgs))
	  (force-ll (prolog:prolog `(kitchen-knowledge:cooking-ingredients (cook ,subst-A from) ?C)))))

(defun get-serving-ingredients (subst-A)
  (mapcar (lambda (bdgs)
            (cut:substitute-vars '?C bdgs))
	  (force-ll (prolog:prolog `(kitchen-knowledge:serving-ingredients (serv ,subst-A with) ?C)))))







;;It isn't working :(
;;(defun get-suitable-container (subst-A subst-B)
;;  (loop for bdg in (force-ll (prolog:prolog `(popcorn-knowledge:suitable-container (fry ,subst-A in ,subst-B) ?C))) 
;;    (cut:substitute-vars '?C bdg)))

;; To call the function: 
;; (popcorn-knowledge:get-cooking-container 'cooking:corn 'cooking:oil)
;; (popcorn-knowledge:get-cooking-container (car (cooking:get-cooking-ingredients 'cooking:popcorn)))
