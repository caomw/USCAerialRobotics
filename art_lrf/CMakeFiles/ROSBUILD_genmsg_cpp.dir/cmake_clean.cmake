FILE(REMOVE_RECURSE
  "msg_gen"
  "src/art_lrf/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/art_lrf/Lines.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
