project(SimulatorSpatzInput)

file(GLOB SOURCES
    *.cpp
)

adtf_add_filter(${PROJECT_NAME} ${SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${ADTF_PLB_PATH}
)

target_link_libraries(${PROJECT_NAME}
    opencv_core
    opencv_imgproc
    LoggingLib
    SharedMemLib
)
