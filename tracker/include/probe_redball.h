/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PROBE_REDBALL_H
#define PROBE_REDBALL_H

#include <RobotsIO/Utils/Probe.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>

#include <yarp/sig/Vector.h>


class ProbeRedball : public RobotsIO::Utils::YarpBufferedPort<yarp::sig::Vector>,
                     public RobotsIO::Utils::Probe
{
public:
    ProbeRedball(const std::string& port_name);

    ProbeRedball(const std::string& port_name, const double& max_x, const double& radius);

private:
    void on_new_data() override;

    yarp::sig::Vector data_;

    double max_x_;

    double radius_;

    const bool enforce_limits_ = false;

    const std::string log_name_ = "ProbeRedball";
};

#endif /* PROBE_REDBALL_H */
