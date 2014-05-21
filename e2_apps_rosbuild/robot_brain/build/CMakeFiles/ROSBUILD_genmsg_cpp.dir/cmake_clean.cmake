FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/robot_brain/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/robot_brain/HighLevelData.h"
  "../msg_gen/cpp/include/robot_brain/WheelData.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
