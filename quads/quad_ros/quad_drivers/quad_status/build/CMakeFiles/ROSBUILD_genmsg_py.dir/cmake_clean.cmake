FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/quad_status/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/quad_status/msg/__init__.py"
  "../src/quad_status/msg/_Status.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
