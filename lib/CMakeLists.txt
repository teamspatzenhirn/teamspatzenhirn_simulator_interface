project(SharedMemLib)

file(GLOB SOURCES
    *.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    SpatzLib
)

default_install(${PROJECT_NAME})
