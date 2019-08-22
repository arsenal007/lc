file(REMOVE_RECURSE
  "liblcusb2.dll"
  "liblcusb2.dll.a"
  "liblcusb2.dll.manifest"
  "liblcusb2.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C CXX)
  include(CMakeFiles/lcusb2.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
