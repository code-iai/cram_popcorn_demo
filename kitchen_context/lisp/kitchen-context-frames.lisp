;;; Copyright (c) 2016, 2016 Gheorghe Lisca <lisca@cs.uni-bremen.de>
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

(in-package :kitchen-context)

(defun broadcast-objects-initial-locations ()

  ;; (parse-gazebo-sdf-file-and-broadcast-statically-initial-poses
  ;;  "chemlab_context" "worlds/chemlab.world")

  ;; (parse-gazebo-sdf-file-and-broadcast-statically-initial-poses
  ;;  "chemlab_context" "worlds/objects_appearing_in_tf_only.world")

  ;; (parse-gazebo-sdf-file-and-broadcast-statically-initial-poses
  ;;  "chemlab_context" "worlds/chemlab_objects_test.world")

  ;; (parse-gazebo-sdf-file-and-broadcast-statically-initial-poses
  ;;  "chemlab_context" "worlds/acat_objects_appearing_in_tf_only.world")
  )

(defun broadcast-objects-offsets-frames ()
  ;; (broadcast-robot-base-standing-frames)
  
  ;; (broadcast-robot-arms-postures-frames)
  
  ;; (broadcast-bottle-500ml-frames)

  ;; (broadcast-bottle-cap-frames)
  ;; (broadcast-bottle-cap-0-frames)
  ;; (broadcast-bottle-cap-1-frames)
  
  ;; (publish-bottle-250ml-frames)
  ;; (publish-bottle-250ml-0-frames)
  ;; (publish-bottle-250ml-1-frames)

  ;; (broadcast-flask-250ml-frames)
  ;; (publish-flask-250ml-0-frames)
  ;; (broadcast-flask-250ml-1-frames)

  ;; (broadcast-flask-400ml-frames)
  ;; (publish-flask-400ml-0-frames)
  ;; (broadcast-flask-400ml-1-frames)

  ;; (broadcast-flask-600ml-frames)
  ;; (broadcast-flask-600ml-0-frames)
  ;; (broadcast-flask-600ml-1-frames)

  ;; (publish-mixer-ikamag-frames)

  ;; (publish-pipette-accumax-frames)

  ;; (broadcast-pipette-tips-box-frames)

  ;; (broadcast-trash-box-frames)
  )

;; (defvar *new-include* nil
;;   "Flags that the XML parser is inside an `include tag.")

;; (defvar *new-frame-name* nil
;;   "Flags that the XML parser is inside an `name tag.")

;; (defvar *new-pose* nil
;;   "Flags that the XML parser is inside a pose tag.")

;; (defvar *new-stamped-transform*
;;   (make-instance
;;       'cl-transforms-stamped:transform-stamped
;;     :stamp 0.0
;;     :frame-id "map"
;;     :child-frame-id ""
;;     :translation (cl-transforms:make-identity-vector)
;;     :rotation  (cl-transforms:make-identity-rotation))
;;   "Holds the transform stamped which will be statically published. The
;;   `child-frame-id is wrapped into a `name tag and the transform is
;;   wrapped into a `pose tag. This is the main reason we need a global
;;   accumulator variable.")

;; (defun xml-new-element-hook (name attributes seed)
;;   "The XML parser calls this function every time it enters a new tag."
;;   (declare (ignore attributes))
;;   (let ((new-seed (cons (1+ (car seed)) (1+ (cdr seed)))))
;;     (if (string-equal name "include")
;;         (setf *new-include* t))
;;     (if (string-equal name "name")
;;         (if *new-include*
;;             (setf *new-frame-name* t)))
;;     (if (string-equal name "pose")
;;         (if *new-include*
;;             (setf *new-pose* t)))
;;     new-seed))

;; (defun xml-finish-element-hook (name attributes parent-seed seed)
;;   "The XML parser calls this function every time it leaves a tag."
;;   (declare (ignore attributes))
;;   (declare (ignore parent-seed))
;;   (let ((new-seed (cons (1- (car seed)) (1+ (cdr seed)))))
;;     (if (string-equal name "include")
;;         (setf *new-include* nil))
;;     (if (string-equal name "name")
;;         (setf *new-frame-name* nil))
;;     (if (string-equal name "pose")
;;         (setf *new-pose* nil))
;;     new-seed))

;; (defun xml-text-hook (parsed-string seed)
;;   "The XML parser calls this function each time it encounters text
;;   after entering a tag.
;;   This function sets the `child-frame-id and
;;   the `transform of the stamped transform which will be statically
;;   published."
;;   (let ((new-seed (cons (car seed) (1+ (cdr seed)))))
;;     ;; set the child-frame of stamped transform
;;     (if *new-frame-name*
;;         (with-slots ((new-frame-id cl-transforms-stamped:child-frame-id)) *new-stamped-transform*
;;           (setf new-frame-id parsed-string)))

;;     ;; set the transform of the stamped transform
;;     (if *new-pose*
;;         (if *new-include*
;;             (progn
;;               ;; parse the string between `pose tags into position and location.
;;               (with-input-from-string (pose-string parsed-string)
;;                 (let ((x-new (read pose-string))
;;                       (y-new (read pose-string))
;;                       (z-new (read pose-string))
;;                       (ax-new (read pose-string))
;;                       (ay-new (read pose-string))
;;                       (az-new (read pose-string)))

;;                   (with-slots ((new-child-frame-id cl-transforms-stamped:child-frame-id)) *new-stamped-transform*

;;                     (cl-transforms-stamped:start-publishing-transforms-static-globaly

;;                      ;; the initial location of the object
;;                      (make-instance
;;                          'cl-transforms-stamped:transform-stamped
;;                        :stamp 0.0
;;                        :frame-id "map"
;;                        :child-frame-id (concatenate 'string
;;                                                     (slot-value *new-stamped-transform* 'cl-transforms-stamped:child-frame-id)
;;                                                     "_initial_frame")
;;                        :translation (cl-transforms:make-3d-vector x-new y-new z-new)
;;                        :rotation (cl-transforms:euler->quaternion :ax ax-new :ay ay-new :az az-new))

;;                      ;; the frame of the object
;;                      (make-instance
;;                          'cl-transforms-stamped:transform-stamped
;;                        :stamp 0.0
;;                        :frame-id "map"
;;                        :child-frame-id (concatenate 'string
;;                                                     (slot-value *new-stamped-transform* 'cl-transforms-stamped:child-frame-id)
;;                                                     "_frame")
;;                        :translation (cl-transforms:make-3d-vector x-new y-new z-new)
;;                        :rotation (cl-transforms:euler->quaternion :ax ax-new :ay ay-new :az az-new))))))

;;               ;; reinitialize the accumulator *new-stamped-transform*
;;               (setf *new-stamped-transform*
;;                     (make-instance
;;                         'cl-transforms-stamped:transform-stamped
;;                       :stamp 0.0
;;                       :frame-id "map"
;;                       :child-frame-id ""
;;                       :translation (cl-transforms:make-identity-vector)
;;                       :rotation (cl-transforms:make-identity-rotation))))))
;;     new-seed))

;; (defun parse-gazebo-sdf-file-and-broadcast-statically-initial-poses (package-name file-name)
;;   "Gets as parameters the ROS package and the path (within the
;;   package) of the Gazebo SDF file holding the initial state of the Gazebo world.
;;   The poses which are read from this SDF file are considered to be realative to
;;   the map frame."
;;   (with-open-file
;;       (gazebo-sdf-file-stream
;;        (concatenate
;;         'string
;;         (namestring
;;          (roslisp::ros-package-path package-name))
;;         file-name))
;;     (s-xml:start-parse-xml gazebo-sdf-file-stream
;;                            (make-instance 's-xml:xml-parser-state
;;                              :seed (cons 0 0)
;;                              ;; seed car is xml element nesting level
;;                              ;; seed cdr is ever increasing from element to element
;;                              :new-element-hook #'xml-new-element-hook
;;                              :finish-element-hook #'xml-finish-element-hook
;;                              :text-hook #'xml-text-hook))))

;; The fake perception stuff.

(defparameter *cl-tf2-buffer-client* nil)

(defparameter *cl-tf2-buffer-client-timeout* 3.0)

(defun create-cl-tf2-buffer-client ()
  (setf *cl-tf2-buffer-client*
        (make-instance 'cl-tf2:buffer-client)))

(roslisp-utilities:register-ros-init-function
 create-cl-tf2-buffer-client)

(defun lookup-object-frame (frame-name)

  (roslisp:ros-info 'kitchen-context-designators
                    "Asking TF2 about the \"~A\" object frame" frame-name)

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
                    (roslisp:ros-warn 'chemlab-context-designators "cl-transforms-stamped:transform-stamped-error !!!")
                    (return-from tf-querry NIL)))

                (cl-transforms-stamped:lookup-error ()
                  (progn
                    (roslisp:ros-warn 'chemlab-context-designators "cl-transforms-stamped:lookup-error !!!")
                    (return-from tf-querry NIL)))))))

      (if transform-stamped
          (return transform-stamped)
          (roslisp:ros-warn 'chemlab-context-designators
                            "waited ~A seconds and no transform ~A -> ~A found ... retrying in 0.1 seconds ..."
                            *cl-tf2-buffer-client-timeout* "map" frame-name))

      (sleep 0.1))))

(defun start-fake-vision ()
  "The fake vision looks up the frame of the object trough cl-tf2."
  (roslisp:def-service-callback
      iai_robosherlock_msgs-srv:perceiveobject (object_name)

    ;;TODO(lisca): lookup the requested frames and return them!
    (roslisp:make-response
     :object_transf (cl-transforms-stamped:to-msg
                     (cond  ;; ((equal object_name "tool_holder")
                            ;;  (lookup-object-frame "tool_holder_frame"))

                            ;; ((equal object_name "tool_holder_rack_0")
                            ;;  (lookup-object-frame "tool_holder_rack_0_frame"))

                            ;; ((equal object_name "tool_holder_rack_1")
                            ;;  (lookup-object-frame "tool_holder_rack_1_frame"))

                            ;; ((equal object_name "spoon_rounded")
                            ;;  (lookup-object-frame "spoon_rounded_frame"))

                            ;; ((equal object_name "spoon_flatten")
                            ;;  (lookup-object-frame "spoon_flatten_frame"))

                            ;; ((equal object_name "red_bowl")
                            ;;  (lookup-object-frame "red_bowl_frame"))

                            ;; ((equal object_name "yellow_bowl")
                            ;;  (lookup-object-frame "yellow_bowl_frame"))

                            ;; ((equal object_name "tray")
                            ;;  (lookup-object-frame "tray_frame"))

                            ;; ((equal object_name "tomato_sauce")
                            ;;  (lookup-object-frame "tomato_sauce_frame"))

                            ;; ((equal object_name "cheese")
                            ;;  (lookup-object-frame "cheese_frame"))

                            ;; ((equal object_name "pizza")
                            ;;  (lookup-object-frame "pizza_frame"))

                            ((equal object_name "pot")
                             (lookup-object-frame "pot_frame"))

                            ((equal object_name "lid")
                             (lookup-object-frame "lid_frame"))

                            ((equal object_name "salt_cellar")
                             (lookup-object-frame "salt_cellar_frame"))

                            ((equal object_name "small_bowl")
                             (lookup-object-frame "small_bowl_frame"))

                            ((equal object_name "deep_plate")
                             (lookup-object-frame "deep_plate_frame"))

                            ((equal object_name "stove_table_drawer_left")
                             (lookup-object-frame "stove_table_drawer_left_frame"))

                            ((equal object_name "stove_table_drawer_right")
                             (lookup-object-frame "stove_table_drawer_right_frame"))

                            ((equal object_name "stove_table")
                             (lookup-object-frame "stove_table_frame"))

                            ((equal object_name "knob_on")
                             (lookup-object-frame "knob_on_frame"))

                            ((equal object_name "knob_off")
                             (lookup-object-frame "knob_off_frame"))


                            (t (roslisp:ros-info
                                'kitchen-context-designators
                                "The object you requested is not detectable ..."))))))

  (roslisp:register-service
   "/RoboSherlock_popcorn/perceive_kitchen_objects"
   'iai_robosherlock_msgs-srv:perceiveobject)

  (roslisp:ros-info
   'kitchen-context-designators
   "Ready to provide fake vision for kitchen objects ..."))
