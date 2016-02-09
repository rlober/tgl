cmake_minimum_required(VERSION 2.8)
project(TrajectoryGeneratorLibrary)
set(lib_name "tgl")
# set(CMAKE_BUILD_TYPE Release)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


file(GLOB source_files ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB header_files ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
find_package(Eigen REQUIRED)

include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/include
${Eigen_INCLUDE_DIRS}
)

add_library(${lib_name} ${source_files})

target_link_libraries(${lib_name})

if(1)
    add_subdirectory(tests)
endif()


#
# # add a target to generate API documentation with Doxygen
# find_package(Doxygen)
# if(DOXYGEN_FOUND)
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
# add_custom_target(doc
# ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
# WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
# COMMENT "Generating API documentation with Doxygen" VERBATIM
# )
# endif(DOXYGEN_FOUND)
