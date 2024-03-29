cmake_minimum_required(VERSION 3.10...3.22)

project(learning_ros_cpp DESCRIPTION ""
   LANGUAGES CXX
)

# ---- Include guards ----
if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
   message(
      FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
   )
endif()

# ---- General information ----
include(cmake_stuff/versions.cmake REQUIRED)
include(cmake_stuff/options.cmake REQUIRED)
include(cmake_stuff/compilerOptions.cmake REQUIRED)

# ---- Add source files ----

# Note: globbing sources is considered bad practice as CMake's generators may not detect new files
# automatically. Keep that in mind when changing files, or explicitly mention them here.
file(GLOB_RECURSE HDRS_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB_RECURSE SRCS_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# ---- Create library ----

# Note: for header-only libraries change all PUBLIC flags to INTERFACE and create an interface
# target: add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME} SHARED ${HDRS_FILES} ${SRCS_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

target_include_directories(${PROJECT_NAME} PUBLIC
   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include/${PROJECT_NAME}-${PROJECT_VERSION}>
)

# ---- Dependencies ----
include(cmake_stuff/findDependencies.cmake REQUIRED)

# ---- Create executable ----
if(${BUILD_UTILS})
   add_subdirectory(utils)
endif(${BUILD_UTILS})

# ---- General info & Build files ----
include(cmake_stuff/buildFiles.cmake REQUIRED)
include(cmake_stuff/printInfo.cmake REQUIRED)
