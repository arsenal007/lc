file(REMOVE_RECURSE
  "liblcusb.a"
  "liblcusb.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C CXX)
  include(CMakeFiles/lcusb.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
