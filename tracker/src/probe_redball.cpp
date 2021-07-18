/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <probe_redball.h>

#include <Eigen/Dense>

#include <RobotsIO/Utils/Data.h>
#include <RobotsIO/Utils/any.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::sig;


ProbeRedball::ProbeRedball(const std::string& port_name) :
    YarpBufferedPort<Vector>(port_name)
{}


void ProbeRedball::on_new_data()
{
    VectorXd data = any_cast<VectorXd>(get_data());

    data_.resize(7);

    /* x, y and z coordinates. */
    data_(0) = data[0];
    data_(1) = data[1];
    data_(2) = data[2];

    /* Estimate validity. */
    data_(6) = 1.0;

    /* The following are not actually used by the RedBall demo. */

    /* Likelihood */
    data_(3) = 1.0;

    /* U, V coordinates */
    data_(4) = 0.0;
    data_(5) = 0.0;

    this->send_data(data_);
}
