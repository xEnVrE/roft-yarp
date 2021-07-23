/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBCARTESIAN_H
#define ICUBCARTESIAN_H

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <string>


class iCubCartesian
{
public:

    iCubCartesian(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix);

    void close();

    bool enable_torso(const bool& enable_yaw, const bool& enable_pitch, const bool& enable_roll);

    bool enable_torso_limits(const std::string& torso_part, const double& min, const double& max);

    bool go_to_pose(const yarp::sig::Vector& position, const yarp::sig::Vector& orientation, const double& trajectory_time);

    bool go_to_pose_stream(const yarp::sig::Vector& position, const yarp::sig::Vector& orientation, const double& trajectory_time);

    bool go_to_position(const yarp::sig::Vector& position, const double& trajectory_time);

    bool go_to_position_stream(const yarp::sig::Vector& position, const double& trajectory_time);

    bool restore_context();

    bool store_context();

    bool stop();

private:
    yarp::os::Network yarp_;

    std::string laterality_;

    /**
     * Driver.
     */
    yarp::dev::PolyDriver driver_cartesian_;

    /**
     * View.
     */
    yarp::dev::ICartesianControl* controller_;

    /**
     * Contexts.
     */
    int initial_context_;

    int current_context_;

    /**
     * Log names to be used in messages printed by the class.
     */

    const std::string log_name_ = "iCubCartesian";
};

#endif /* ICUBCARTESIAN_H */
