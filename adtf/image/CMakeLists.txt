project(SimulatorImageFilter)


file(GLOB SOURCES
    *.cpp
)

#list (APPEND CMAKE_MODULE_PATH "/usr/src/SMARTEKvision/CameraSuite/CMakePackages")
#find_package(CSPackage REQUIRED)

adtf_add_filter(${PROJECT_NAME} ${SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${ADTF_PLB_PATH}
)
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-error=pointer-arith)
target_link_libraries(${PROJECT_NAME}
    opencv_core
    opencv_imgproc
    LoggingLib
    SharedMemLib
)