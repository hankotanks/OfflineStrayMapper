cmake_minimum_required(VERSION 3.14)

project(StrayMapper VERSION 1.0)

set(STRAY_MAPPER_DIR_SRC src)
set(STRAY_MAPPER_DIR_INC include)
set(STRAY_MAPPER_DIR_RTABMAP rtabmap)

set(CMAKE_CXX_VERSION 17)
set(CMAKE_CXX_VERSION_REQUIRED True)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Qt5 COMPONENTS Widgets)

add_subdirectory(rtabmap)

set(moc_srcs ${STRAY_MAPPER_DIR_INC}/MapBuilder.h)

add_executable(
  StrayMapper 
  ${STRAY_MAPPER_DIR_SRC}/main.cpp
  ${STRAY_MAPPER_DIR_SRC}/StrayMapper.cpp
  ${moc_srcs})
target_include_directories(
  StrayMapper PRIVATE 
  ${STRAY_MAPPER_DIR_INC} 
  SYSTEM ${STRAY_MAPPER_DIR_RTABMAP}/corelib/include
  SYSTEM ${STRAY_MAPPER_DIR_RTABMAP}/utilite/include)
target_link_libraries(
  StrayMapper 
  rtabmap::gui)

set_target_properties(StrayMapper PROPERTIES AUTOUIC ON AUTOMOC ON AUTORCC ON)
