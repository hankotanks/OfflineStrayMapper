cmake_minimum_required(VERSION 3.14)

project(StrayMapper VERSION 1.0)

set(STRAY_MAPPER_DIR_SRC src)
set(STRAY_MAPPER_DIR_INC include)
set(STRAY_MAPPER_DIR_RTABMAP rtabmap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

add_definitions(-DPY_PATH_SCRIPTS="${CMAKE_SOURCE_DIR}/scripts")
add_definitions(-DPY_PATH_ENV="${CMAKE_SOURCE_DIR}/env")
add_definitions(-DPY_PATH_PROMPTDA_MODULE="${CMAKE_SOURCE_DIR}/PromptDA")
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(rtabmap)

set(moc_srcs ${STRAY_MAPPER_DIR_INC}/MapBuilder.h)

add_executable(
  StrayMapper 
  ${STRAY_MAPPER_DIR_SRC}/main.cpp
  ${STRAY_MAPPER_DIR_SRC}/StrayMapper.cpp
  ${STRAY_MAPPER_DIR_SRC}/util/PyScript.cpp
  ${moc_srcs}
)

if(UPSCALE_NO_PROMPTDA)
   target_compile_definitions(StrayMapper PRIVATE UPSCALE_NO_PROMPTDA)
endif()

target_include_directories(
  StrayMapper PRIVATE 
  ${STRAY_MAPPER_DIR_INC} 
  SYSTEM ${STRAY_MAPPER_DIR_RTABMAP}/corelib/include
  SYSTEM ${STRAY_MAPPER_DIR_RTABMAP}/utilite/include
  SYSTEM ${Python3_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  StrayMapper 
  Python3::Python
  Python3::Module
  Qt5::Core
  Qt5::Widgets
  rtabmap::core
  rtabmap::gui
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

set_target_properties(StrayMapper PROPERTIES AUTOUIC ON AUTOMOC ON AUTORCC ON)