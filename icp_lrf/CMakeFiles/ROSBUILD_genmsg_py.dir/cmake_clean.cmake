FILE(REMOVE_RECURSE
  "msg_gen"
  "src/icp_lrf/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/icp_lrf/msg/__init__.py"
  "src/icp_lrf/msg/_Lines.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
