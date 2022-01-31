project(SharedMemLib)

set(SOURCES shmcomm.cpp SpatzMultiplexer.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        SpatzLib
        MsgInLib
        )

default_install(${PROJECT_NAME})

# has to be a separate lib for constants since not all needed functions are constexpr
project(SimCameraCalib)

add_library(${PROJECT_NAME} SimCameraCalib.cpp)
target_link_libraries(${PROJECT_NAME} opencv_core)

default_install(${PROJECT_NAME})
