#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME roft-track-n-grasp)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           dev
                           os
                           sig
)

# find_package(ICUB REQUIRED COMPONENTS
#                            iKin
# )

# Eigen
# find_package(Eigen3 REQUIRED)

# Robots I/O facilities
# find_package(RobotsIO REQUIRED)

# Object Tracking library
find_package(OTL REQUIRED)

# Thrift
set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/idl.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

# Executable
add_executable(${EXE_TARGET_NAME}
               include/module.h
               include/iCubGaze.h
               src/main.cpp
               src/module.cpp
               src/iCubGaze.cpp
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
)

target_include_directories(${EXE_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         # Eigen3::Eigen
                                         # OTL::OTL
                                         # RobotsIO::RobotsIO
                                         YARP::YARP_dev
                                         YARP::YARP_init
                                         YARP::YARP_os
                                         YARP::YARP_sig
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Configuration files
yarp_install(FILES app/roft-track-n-grasp.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES app/conf/config.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
