project(SimulatorVisualizationOutput)

file(GLOB SOURCES
    *.cpp
)

adtf_add_filter(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
    opencv_core
    opencv_imgproc
    LoggingLib
    SharedMemLib
)
