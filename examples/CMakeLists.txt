######################################################################################
# Build function for examples
######################################################################################

SET(LIBNIFALCON_EXE_PO_LINK_LIBS ${LIBNIFALCON_EXE_LINK_LIBS} nifalcon_cli_base)

######################################################################################
# Build function for barrow_mechanics
######################################################################################

SET(SRCS barrow_mechanics/main.cpp)

BUILDSYS_BUILD_EXE(
  NAME barrow_mechanics
  SOURCES "${SRCS}" 
  CXX_FLAGS "${DEFINE}" 
  LINK_LIBS "${LIBNIFALCON_EXE_LINK_LIBS}" 
  LINK_FLAGS FALSE 
  DEPENDS nifalcon
  SHOULD_INSTALL TRUE
  )

######################################################################################
# Build function for findfalcons
######################################################################################

SET(SRCS 
  findfalcons/findfalcons.cpp
  )

BUILDSYS_BUILD_EXE(
  NAME findfalcons
  SOURCES "${SRCS}" 
  CXX_FLAGS "${DEFINE}" 
  LINK_LIBS "${LIBNIFALCON_EXE_LINK_LIBS}" 
  LINK_FLAGS FALSE 
  DEPENDS nifalcon
  SHOULD_INSTALL TRUE
  )

######################################################################################
# Build function for findfalcons_multi
######################################################################################

SET(SRCS 
  findfalcons_multi/findfalcons_multi.cpp
  )

BUILDSYS_BUILD_EXE(
  NAME findfalcons_multi
  SOURCES "${SRCS}" 
  CXX_FLAGS "${DEFINE}" 
  LINK_LIBS "${LIBNIFALCON_EXE_LINK_LIBS}" 
  LINK_FLAGS FALSE 
  DEPENDS nifalcon
  SHOULD_INSTALL TRUE
  )

######################################################################################
# Build function for falcon_led
######################################################################################

SET(SRCS
  falcon_led/falcon_led.cpp
)
  
BUILDSYS_BUILD_EXE(
    NAME falcon_led
    SOURCES "${SRCS}" 
    CXX_FLAGS FALSE
    LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}" 
    LINK_FLAGS FALSE 
    DEPENDS nifalcon_cli_base
    SHOULD_INSTALL TRUE
)

######################################################################################
# Build function for cartesian_coords
######################################################################################
SET(SRCS
  cartesian_coords/cartesian_coords.cpp
)

BUILDSYS_BUILD_EXE(
    NAME cartesian_coords
    SOURCES "${SRCS}"
    CXX_FLAGS "${DEFINE}"
    LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}"
    LINK_FLAGS FALSE
    DEPENDS nifalcon_cli_base
    SHOULD_INSTALL TRUE
)

# requires installing zmq along with C++ bindings: https://github.com/zeromq/cppzmq
find_package(cppzmq)
target_link_libraries(cartesian_coords cppzmq)

######################################################################################
# Build function for pd_ontroller
######################################################################################
SET(SRCS
  pd_controller/pd_controller.cpp
  pd_controller/util.cpp
)

BUILDSYS_BUILD_EXE(
    NAME pd_controller
    SOURCES "${SRCS}"
    CXX_FLAGS "${DEFINE}"
    LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}"
    LINK_FLAGS FALSE
    DEPENDS nifalcon_cli_base
    SHOULD_INSTALL TRUE
)

# requires installing zmq along with C++ bindings: https://github.com/zeromq/cppzmq
find_package(cppzmq)
target_link_libraries(pd_controller cppzmq)


######################################################################################
# Build function for displacement_based_force
######################################################################################
SET(SRCS
  displacement_based_force/displacement_based_force.cpp
  displacement_based_force/util.cpp
)

BUILDSYS_BUILD_EXE(
    NAME displacement_based_force
    SOURCES "${SRCS}"
    CXX_FLAGS "${DEFINE}"
    LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}"
    LINK_FLAGS FALSE
    DEPENDS nifalcon_cli_base
    SHOULD_INSTALL TRUE
)

# requires installing zmq along with C++ bindings: https://github.com/zeromq/cppzmq
find_package(cppzmq)
target_link_libraries(displacement_based_force cppzmq)

######################################################################################
# Build function for falcon_json
######################################################################################
SET(SRCS
  falcon_json/falcon_json.cpp
  ../util/util.cpp
)

BUILDSYS_BUILD_EXE(
    NAME falcon_json
    SOURCES "${SRCS}"
    CXX_FLAGS "${DEFINE}"
    LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}"
    LINK_FLAGS FALSE
    DEPENDS nifalcon_cli_base
    SHOULD_INSTALL TRUE
)

# requires installing zmq along with C++ bindings: https://github.com/zeromq/cppzmq
find_package(cppzmq)
find_package(nlohmann_json 3.2.0 REQUIRED)
target_link_libraries(falcon_json cppzmq)
target_link_libraries(falcon_json PRIVATE nlohmann_json::nlohmann_json)

######################################################################################
# Build function for falcon_test_cli
######################################################################################

SET(SRCS
    falcon_test_cli/falcon_test_cli.cpp 
    falcon_test_cli/FalconTestBase.cpp 
    falcon_test_cli/FalconLoopTimeTest.cpp 
    falcon_test_cli/FalconCubeTest.cpp 
    falcon_test_cli/FalconSphereTest.cpp 
    falcon_test_cli/FalconWallTest.cpp 
    falcon_test_cli/FalconColorTest.cpp
)

BUILDSYS_BUILD_EXE(
    NAME falcon_test_cli
    SOURCES "${SRCS}" 
    CXX_FLAGS FALSE
    LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}" 
    LINK_FLAGS FALSE 
    DEPENDS nifalcon_cli_base
    SHOULD_INSTALL TRUE
)

######################################################################################
# Build function for falcon_mouse
######################################################################################

  IF(WIN32)
	SET(MOUSE_SRCS falcon_mouse/falcon_mouse.cpp falcon_mouse/falcon_mouse_win.cpp)
	SET(MOUSE_LINK_LIBS ${LIBNIFALCON_EXE_PO_LINK_LIBS})
  ELSEIF(APPLE)
	SET(MOUSE_SRCS falcon_mouse/falcon_mouse.cpp falcon_mouse/falcon_mouse_osx.cpp)
	SET(MOUSE_LINK_LIBS ${LIBNIFALCON_EXE_PO_LINK_LIBS})
	SET(MOUSE_LINK_FLAGS "-framework Carbon -framework ApplicationServices")
  ELSEIF(UNIX)
	FIND_PACKAGE(X11)
	IF(X11_FOUND AND X11_XTest_LIB)
	  INCLUDE_DIRECTORIES(${X11_INCLUDE_DIR})
	  SET(MOUSE_SRCS falcon_mouse/falcon_mouse.cpp falcon_mouse/falcon_mouse_x11.cpp)      
	  SET(MOUSE_LINK_LIBS ${LIBNIFALCON_EXE_PO_LINK_LIBS} ${X11_LIBRARIES} ${X11_XTest_LIB}) 
	ELSE(X11_FOUND AND X11_XTest_LIB)
	  MESSAGE(STATUS "Not compiling falcon_mouse example - X11 compiling requires the XTest library")
	ENDIF(X11_FOUND AND X11_XTest_LIB)
  ENDIF(WIN32)
  IF(MOUSE_SRCS)
	BUILDSYS_BUILD_EXE(
      NAME falcon_mouse
      SOURCES "${MOUSE_SRCS}" 
      LINK_LIBS "${MOUSE_LINK_LIBS}" 
      LINK_FLAGS "${MOUSE_LINK_FLAGS}" 
      CXX_FLAGS FALSE
      DEPENDS nifalcon
      SHOULD_INSTALL TRUE
      )
  ENDIF(MOUSE_SRCS)
