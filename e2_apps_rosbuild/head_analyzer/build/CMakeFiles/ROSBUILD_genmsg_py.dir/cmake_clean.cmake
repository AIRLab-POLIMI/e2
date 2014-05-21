FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/head_analyzer/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/head_analyzer/msg/__init__.py"
  "../src/head_analyzer/msg/_MoveDataMSG.py"
  "../src/head_analyzer/msg/_HeadDataMSG.py"
  "../src/head_analyzer/msg/_EyesDataMSG.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
