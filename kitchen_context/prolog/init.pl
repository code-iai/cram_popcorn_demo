:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).

:- use_module(library('knowrob_owl')).

:- owl_parser:owl_parse('package://kitchen_context/owl/stove_table_semantic_map.owl').
