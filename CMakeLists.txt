project(GeomProc)
cmake_minimum_required(VERSION 2.8)

###############################
# set flags
###############################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-extensions -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

###############################
# set output path
###############################
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib" CACHE PATH
    "Output directory for library" )
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin" CACHE PATH
     "Output directory for executables" )

# Set include directory
include_directories(/usr/local/include/eigen3)

######################
# boost
######################
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(Boost_INCLUDE_DIR /usr/include)
  set(Boost_LIBRARY_DIR /usr/lib)
  set(BOOST_LIBRARYDIR /usr/lib)
  link_directories(/usr/lib)
  get_directory_property(OUT_VAR LINK_DIRECTORIES)
  message(STATUS "LINK DIR: ${OUT_VAR}")
endif()

FIND_PACKAGE(Boost COMPONENTS filesystem thread program_options system log REQUIRED)
if (Boost_FOUND)
  message(STATUS "Found 'boost library'")
  include_directories(${Boost_INCLUDE_DIR})
  message(STATUS "  boost lib dir:${Boost_LIBRARY_DIRS}")
  message(STATUS "  boost include dir:${Boost_INCLUDE_DIR}")
  message(STATUS "  libs:  ${Boost_LIBRARIES}" )
else()
  message(FATAL_ERROR "Couldn't find Boost library.")
endif()

##########################
# opencv
##########################
set(OpenCV_DIR "/usr/local/opencv/share/OpenCV/OpenCVConfig.cmake"
    CACHE PATH "The path where OpenCVConfig.cmake is placed")
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message(STATUS "Found 'opencv library' ${OpenCV_VERSION}")
  include_directories(${OpenCV_INCLUDE_DIRS})
else()
  message( SEND_ERROR "You can't do that" )
endif()

##########################
# tetgen
##########################
add_subdirectory("external/tetgen")

##########################
# triangle
##########################
add_subdirectory("external/triangle")


##########################
# geom
##########################
add_subdirectory("src/geom")

##########################
# io
##########################
add_subdirectory("src/io")

##########################
# render
##########################
add_subdirectory("src/render")

##########################
# sampling
##########################
add_subdirectory("src/algorithm")

##########################
# test
##########################
#add_subdirectory("src/tests")
