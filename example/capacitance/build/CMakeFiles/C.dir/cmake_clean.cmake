file(REMOVE_RECURSE
  "C.exe"
  "C.exe.manifest"
  "C.pdb"
  "libC.dll.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/C.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
