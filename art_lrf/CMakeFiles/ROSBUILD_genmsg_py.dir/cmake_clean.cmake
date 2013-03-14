FILE(REMOVE_RECURSE
  "msg_gen"
  "src/art_lrf/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/art_lrf/msg/__init__.py"
  "src/art_lrf/msg/_Lines.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
