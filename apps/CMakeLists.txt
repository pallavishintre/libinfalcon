######################################################################################
# Build function for apps
######################################################################################

SET(LIBNIFALCON_EXE_PO_LINK_LIBS ${LIBNIFALCON_EXE_LINK_LIBS} nifalcon_cli_base)

######################################################################################
# Build function for util
######################################################################################

#SET(SRCS
#  util/util.cpp
#)

#BUILDSYS_BUILD_EXE(
#  NAME util
#  SOURCES "${SRCS}"
#  CXX_FLAGS "${DEFINE}"
#  LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}"
#  LINK_FLAGS FALSE
#  DEPENDS nifalcon_cli_base
#  SHOULD_INSTALL TRUE
#)

######################################################################################
# Build function for test_falcon_connection
######################################################################################

SET(SRCS 
  test_falcon_connection/test_falcon_connection.cpp
  ../util/util.cpp
)

BUILDSYS_BUILD_EXE(
  NAME test_falcon_connection
  SOURCES "${SRCS}" 
  CXX_FLAGS "${DEFINE}" 
  LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}" 
  LINK_FLAGS FALSE 
  DEPENDS nifalcon_cli_base
  SHOULD_INSTALL TRUE
)

######################################################################################
# Build function for test_falcon_impedance
######################################################################################

SET(SRCS
  test_falcon_impedance/test_falcon_impedance.cpp
  ../util/util.cpp
)

BUILDSYS_BUILD_EXE(
  NAME test_falcon_impedance
  SOURCES "${SRCS}"
  CXX_FLAGS "${DEFINE}"
  LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}"
  LINK_FLAGS FALSE
  DEPENDS nifalcon_cli_base
  SHOULD_INSTALL TRUE
)

######################################################################################
# Build function for falcon_practice
######################################################################################

SET(SRCS
  falcon_practice/falcon_practice.cpp
  ../util/util.cpp
)

BUILDSYS_BUILD_EXE(
  NAME falcon_practice
  SOURCES "${SRCS}"
  CXX_FLAGS "${DEFINE}"
  LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}"
  LINK_FLAGS FALSE
  DEPENDS nifalcon_cli_base
  SHOULD_INSTALL TRUE
)

# requires installing zmq along with C++ bindings: https://github.com/zeromq/cppzmq
find_package(cppzmq)
target_link_libraries(falcon_practice cppzmq)

######################################################################################
# Build function for falcon_practice
######################################################################################

SET(SRCS
  falcon_client/falcon_client.cpp
  ../util/util.cpp
)

BUILDSYS_BUILD_EXE(
  NAME falcon_client
  SOURCES "${SRCS}"
  CXX_FLAGS "${DEFINE}"
  LINK_LIBS "${LIBNIFALCON_EXE_PO_LINK_LIBS}"
  LINK_FLAGS FALSE
  DEPENDS nifalcon_cli_base
  SHOULD_INSTALL TRUE
)

# requires installing zmq along with C++ bindings: https://github.com/zeromq/cppzmq
find_package(cppzmq)
target_link_libraries(falcon_client cppzmq)
