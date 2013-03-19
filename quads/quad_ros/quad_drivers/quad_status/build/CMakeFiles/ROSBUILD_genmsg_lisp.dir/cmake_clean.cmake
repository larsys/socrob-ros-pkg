FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/quad_status/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Status.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Status.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
