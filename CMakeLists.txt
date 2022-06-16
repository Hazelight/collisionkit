cmake_minimum_required(VERSION 3.1...3.22)

project(collisionkit)

# Include CGAL, looking initially at the Optimal_bounding_box example
find_package(CGAL REQUIRED)

find_package(Eigen3 3.1.0 REQUIRED) #(3.1.0 or greater)
include(CGAL_Eigen3_support)
if(NOT TARGET CGAL::Eigen3_support)
  message(
    STATUS "This project requires the Eigen library, and will not be compiled.")
  return()
endif()

# Create a variable to allow developers to set the path to the downloaded sdk.
# no default seeing as it doesn't install to any pre defined location.
set(LXSDK_PATH "" CACHE PATH "Path to root of downloaded LXSDK")

# Get all source and headers for lxsdk
file(GLOB LXSDK_SOURCES ${LXSDK_PATH}/common/*.cpp)
file(GLOB LXSDK_HEADERS ${LXSDK_PATH}/include/*.h?)

# CRT_SECURE_NO_WARNINGS on windows,
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# Should create our library so we can now focus on our own project.
add_library(lxsdk STATIC ${LXSDK_SOURCES})
set_target_properties(lxsdk PROPERTIES LIBRARY_OUTPUT_DIRECTORY lib)
target_include_directories(lxsdk PRIVATE ${LXSDK_PATH}/include)

# This is the plug-in we create, shared makes it on windows to a .dll which is
# what we expect for a plug-in
add_library(collisionkit SHARED "source/collisionkit.cpp")

# include the headers for the sdk
target_include_directories(collisionkit PRIVATE ${LXSDK_PATH}/include ${CGAL_INCLUDE})

target_link_libraries(collisionkit PUBLIC lxsdk CGAL::CGAL CGAL::Eigen3_support)

# we could otherwise here potentially later branch for unix also
set(PLUGIN_DIR "win64")

# Set the output to the folder Modo will search, 
# $<0:> is just to remove any config subfolders like DEBUG and RELEASE
set_target_properties(collisionkit 
  PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/${PLUGIN_DIR}/$<0:>
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/${PLUGIN_DIR}/$<0:> # windows apparently needs this set as well
)
