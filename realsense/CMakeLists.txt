#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

yarp_install(FILES app/roft_realsense.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES app/conf/config_realsense_720p.ini app/conf/config_realsense_480p.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/roft-tracker)
