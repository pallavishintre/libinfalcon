FIND_PACKAGE(SWIG)
FIND_PACKAGE(JNI REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)

IF(NOT SWIG_FOUND)
  RETURN()
ENDIF(NOT SWIG_FOUND)

INCLUDE(${SWIG_USE_FILE})

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH} ../include ${LIBNIFALCON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

SET(CMAKE_SWIG_FLAGS "")

SET(NP_EXE_CPP_DEFINES )
LIBNIFALCON_COMM_VAR_SETUP()

LIST(APPEND NP_EXE_CPP_LINK_LIBS ${LIBNIFALCON_REQ_LIBS})

IF(BUILD_STATIC AND APPLE)
  LIST(APPEND NP_EXE_CPP_LINK_LIBS "-framework IOKit" "-framework CoreFoundation")
ENDIF(BUILD_STATIC AND APPLE)

SET(LIBNIFALCON_INTERFACE libnifalcon_swig.i)

SET_SOURCE_FILES_PROPERTIES(${LIBNIFALCON_INTERFACE} PROPERTIES CPLUSPLUS ON)
SET_SOURCE_FILES_PROPERTIES(../src/FalconDeviceBridge.cpp PROPERTIES COMPILE_FLAGS "-DLIBUSB")

SET(ARGS "-includeall" "-module" "pynifalcon")

SET_SOURCE_FILES_PROPERTIES(${LIBNIFALCON_INTERFACE} PROPERTIES SWIG_FLAGS "${ARGS}")
SET(LIBS_PYTHON ${NP_EXE_CPP_LINK_LIBS} ${PYTHON_LIBRARIES})
SWIG_ADD_MODULE(pynifalcon python ${LIBNIFALCON_INTERFACE} ../src/FalconDeviceBridge.cpp)
SWIG_LINK_LIBRARIES(pynifalcon ${LIBS_PYTHON})

SET(ARGS "-includeall" "-module" "JNIFalcon")

SET_SOURCE_FILES_PROPERTIES(${LIBNIFALCON_INTERFACE} PROPERTIES SWIG_FLAGS "${ARGS}")
SET(LIBS_JNI ${NP_EXE_CPP_LINK_LIBS} ${JNI_LIBRARIES})
SWIG_ADD_MODULE(JNIFalcon java ${LIBNIFALCON_INTERFACE} ../src/FalconDeviceBridge.cpp)
SWIG_LINK_LIBRARIES(JNIFalcon ${LIBS_JNI})
