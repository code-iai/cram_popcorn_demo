(in-package :kitchen-context)

;; simple semantic map query

;; (stove-table::query-semantic-map
;;  "current_object_pose"
;;  "http://knowrob.org/kb/stove_table_semantic_map.owl#StoveTable_RbTLNllQ")

(defun query-semantic-map (predicate-string object-owl-string)
  (json-prolog::with-vars-bound (?x ?y ?z ?qw ?qx ?qy ?qz)
      (json-prolog::lazy-car
       (json-prolog:prolog
        `(,predicate-string
          ,object-owl-string
          '(?x ?y ?z ?qw ?qx ?qy ?qz))
        :package :kitchen-context))

    (cl-tf2:make-pose
     (cl-tf2:make-3d-vector ?x ?y ?z)
     (cl-tf2:make-quaternion ?qx ?qy ?qz ?qw))))


(defun query-semantic-map-2 (object-owl-string)
  (json-prolog::with-vars-bound (?x ?y ?z ?qw ?qx ?qy ?qz)
      (json-prolog::lazy-car
       (json-prolog:prolog
        `("current_object_pose"
          ,object-owl-string
          '(?x ?y ?z ?qw ?qx ?qy ?qz))
        :package :kitchen-context))

    (cl-tf2:make-pose
     (cl-tf2:make-3d-vector ?x ?y ?z)
     (cl-tf2:make-quaternion ?qx ?qy ?qz ?qw))
    (write ?x)
    (write ?y)))


(defun query-next-action (predicate-string last-action-string action-name-string)
  (json-prolog::with-vars-bound (?Action)
      (json-prolog::lazy-car
       (json-prolog:prolog
        `(,predicate-string
          ,last-action-string
          ,action-name-string
          '(?Action))
        :package :kitchen-context))
    (print (type-of ?Action))
    (print ?Action)))


(defun query-next-action-2 (last-action-string action-name-string)
  (json-prolog::with-vars-bound (?Action)
      (json-prolog::lazy-car
       (json-prolog:prolog
        `(,(string "assembly_next_action")
          ,last-action-string
          ,action-name-string
          '(?Action))
        :package :kitchen-context))
    (write ?Action)
    (cl-tf2:make-pose
     (cl-tf2:make-3d-vector ?Action 1 2)
     (cl-tf2:make-quaternion 1 0 0 0)

     )))


