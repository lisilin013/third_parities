# Calling gcc through gcc-5 is necessary as on some Apple systems the commands
# gcc and g++ are symbolic links to Apples' compiler.
set(CMAKE_C_COMPILER gcc-5 CACHE STRING "C compiler" FORCE)
set(CMAKE_CXX_COMPILER g++-5 CACHE STRING "C++ compiler" FORCE)
