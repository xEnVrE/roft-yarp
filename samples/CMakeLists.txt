#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(BUILD_TRACK_N_GRASP "Build tracking with grasping sample." OFF)
option(BUILD_GRASP_VIEWER  "Build grasp candidates viewer." OFF)

if (BUILD_TRACK_N_GRASP)
   add_subdirectory(track-n-grasp)
endif()

if (BUILD_GRASP_VIEWER)
   add_subdirectory(grasp-viewer)
endif()
