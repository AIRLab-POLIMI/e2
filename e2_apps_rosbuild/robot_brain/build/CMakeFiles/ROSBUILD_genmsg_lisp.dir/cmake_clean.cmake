FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/robot_brain/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/HighLevelData.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_HighLevelData.lisp"
  "../msg_gen/lisp/WheelData.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_WheelData.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
