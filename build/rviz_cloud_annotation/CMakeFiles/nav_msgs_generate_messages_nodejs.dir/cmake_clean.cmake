file(REMOVE_RECURSE
  "point_cloud_plane_detection_automoc.cpp"
  "rviz_cloud_annotation_com_automoc.cpp"
  "rviz_cloud_annotation_node_automoc.cpp"
  "rviz_cloud_annotation_plugin_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/nav_msgs_generate_messages_nodejs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
