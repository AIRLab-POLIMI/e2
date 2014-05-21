FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/robot_brain/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/robot_brain/msg/__init__.py"
  "../src/robot_brain/msg/_HighLevelData.py"
  "../src/robot_brain/msg/_WheelData.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
