cmake_minimum_required(VERSION 3.0.0)
project(CppHomework1 VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

# USING GLOB IS OLD STYLE AND NOT RECOMMENDED
# it is better to list all the files explicitly
# used for simplicity
file(GLOB SOURCES "*/*.cpp" "*.cpp")
add_executable(CppHomework1 ${SOURCES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
