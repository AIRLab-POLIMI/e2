FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/head_analyzer/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/MoveDataMSG.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MoveDataMSG.lisp"
  "../msg_gen/lisp/HeadDataMSG.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_HeadDataMSG.lisp"
  "../msg_gen/lisp/EyesDataMSG.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_EyesDataMSG.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
