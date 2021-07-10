#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME roft-realsense)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           os
                           sig
)

# OpenCV
find_package(OpenCV REQUIRED)

# RealSense
find_package(realsense2 REQUIRED)

# RobotsIO
find_package(RobotsIO REQUIRED)

# Executable
add_executable(${EXE_TARGET_NAME} src/main.cpp)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      ${OpenCV_LIBS}
                      realsense2::realsense2
                      RobotsIO::RobotsIO
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Configuration files
yarp_install(FILES app/roft_realsense.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES app/conf/config_realsense_720p.ini app/conf/config_realsense_480p.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/roft-tracker)
