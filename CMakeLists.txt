cmake_minimum_required(VERSION 3.8)
project(teamspatzenhirn_simulator_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(src)

function(default_install target)
  if (NOT TARGET ${target})
    message(SEND_ERROR "default_install expected a target name, but got \"${target}\"")
  endif ()
  install(
          TARGETS ${target}
          EXPORT export_${target}
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
          INCLUDES DESTINATION include
  )
endfunction()

add_subdirectory(src/lib)
add_subdirectory(src/ros)
add_subdirectory(src/test/standalone)

ament_package()
