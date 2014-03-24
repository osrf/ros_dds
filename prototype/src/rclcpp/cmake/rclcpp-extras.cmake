# For older versions of clang, libstdc++ is used by default and we need to
# explicitly request libc++ instead.
if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++0x -stdlib=libc++")
endif ()
