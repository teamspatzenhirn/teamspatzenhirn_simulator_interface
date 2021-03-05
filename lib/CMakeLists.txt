project(SharedMemLib)

file(GLOB SOURCES
    *.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${ADTF_LIB_PATH}
        )


target_link_libraries(${PROJECT_NAME}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    SpatzLib
)
