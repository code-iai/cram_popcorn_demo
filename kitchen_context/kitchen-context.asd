;;;
;;; Copyright (c) 2017 Gheorghe Lisca <lisca@cs.uni-bremen.de>
;;;                    Mihaela Popescu <mihaela.popescu94@gmail.com>
;;;                    Boglárka Erdődi <erdodiboglar@gmail.com>
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

(defsystem kitchen-context
  :author "Gheorghe Lisca, Mihaela Popescu, Boglárka Erdődi"
  :license "BSD"
  :depends-on (plan-library
	       roslisp-utilities
               cl-tf2
	       cram-designators
               cram-language-designator-support
               cram-language
               cram-process-modules
               cram-prolog
               ;;cram-moveit
               cram-json-prolog
               std_srvs-srv
               actionlib)

  :components
  ((:module "lisp"
            :components
            ((:file "package")

             (:file "knowrob-interface"
     	     :depends-on ("package"))

             (:file "kitchen-island-frames"
              :depends-on ("package"))

              (:file "stove-table-frames"
              :depends-on ("package"))

              (:file "stove-table-drawer-left-frames"
               :depends-on ("package"))

              (:file "stove-table-drawer-right-frames"
               :depends-on ("package"))
             
             (:file "tool-holder-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))

             (:file "spoon-rounded-frames"
              :depends-on ("package"
                           "kitchen-island-frames"
                           "tool-holder-frames"))

             (:file "spoon-flatten-frames"
              :depends-on ("package"
                           "kitchen-island-frames"
                           "tool-holder-frames"))

             (:file "red-bowl-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))

             (:file "yellow-bowl-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))

             (:file "green-bowl-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))

             (:file "tray-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))

             (:file "pot-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))

             (:file "lid-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))

              (:file "salt-cellar-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))

              (:file "small-bowl-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))
             
             (:file "deep-plate-frames"
              :depends-on ("package"
                           "kitchen-island-frames"))
             
             (:file "scooping-frames"
              :depends-on ("package"))

             (:file "dispensing-frames"
              :depends-on ("package"))

             (:file "spreading-frames"
              :depends-on ("package"))

             (:file "kitchen-context-frames"
              :depends-on ("package"                           
                           "tool-holder-frames"))

             (:file "tomato-sauce-frames"
              :depends-on ("package"
                           "red-bowl-frames"))

             (:file "cheese-frames"
              :depends-on ("package"
                           "yellow-bowl-frames"))

             (:file "pizza-frames"
              :depends-on ("package"
                           "tray-frames"))

             (:file "location-designators"
              :depends-on ("package"
                           "scooping-frames"
                           "dispensing-frames"
                           "spreading-frames"
                           "tomato-sauce-frames"
                           "cheese-frames"
                           "pizza-frames"
                           "kitchen-context-frames"))
             
             (:file "object-designators"
              :depends-on ("package"
                           
                           "tool-holder-frames"
                           "spoon-rounded-frames"
                           "red-bowl-frames"
                           "yellow-bowl-frames"
                           "tray-frames"
                           
                           "kitchen-context-frames"))
             
             (:file "action-designators"
              :depends-on ("package"))))))

        
