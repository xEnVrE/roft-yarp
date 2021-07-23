/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubGaze.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;


iCubGaze::iCubGaze(const std::string& robot_name, const std::string& port_prefix)
{
    /* Check YARP network. */
    if (!yarp_.checkNetwork())
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));

    Property properties;
    properties.put("device", "gazecontrollerclient");
    properties.put("remote", "/iKinGazeCtrl");
    properties.put("local", "/" + port_prefix + "/gazecontroller");

    /* Open driver. */
    bool ok = driver_gaze_.open(properties) && driver_gaze_.view(gaze_) && (gaze_ != nullptr);

    if (!ok)
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open IGazeControl interface for robot " + robot_name  + "."));

    /* Store actual IGazeControl context. */
    gaze_->storeContext(&gaze_startup_context_);

    /* Disable head roll by default. */
    gaze_->blockNeckRoll(0.0);

    /* Set trajectory times. */
    gaze_->setNeckTrajTime(0.7);
    gaze_->setEyesTrajTime(0.5);

    /* Set home configuration. */
    home_configuration_.resize(3);
    home_configuration_(0) = -1.0;
    home_configuration_(1) = 0.0;
    home_configuration_(2) = 0.3;
}


void iCubGaze::close()
{
    stop();
    gaze_->restoreContext(gaze_startup_context_);

    if (driver_gaze_.isValid())
        driver_gaze_.close();
}


bool iCubGaze::look_at(const Vector& target)
{
    return gaze_->lookAtFixationPointSync(target);
}


bool iCubGaze::look_at_stream(const Vector& target)
{
    return gaze_->lookAtFixationPoint(target);
}


bool iCubGaze::go_home()
{
    int current_context;

    gaze_->storeContext(&current_context);
    gaze_->setNeckTrajTime(3.0);

    look_at(home_configuration_);

    gaze_->restoreContext(current_context);

    return true;
}


void iCubGaze::set_home_configuation(const yarp::sig::Vector& configuration)
{
    home_configuration_ = configuration;
}


bool iCubGaze::stop()
{
    return gaze_->stopControl();
}
