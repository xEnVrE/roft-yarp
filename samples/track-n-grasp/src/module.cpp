/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <module.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>

#include <thread>

using namespace Eigen;
using namespace std::literals::chrono_literals;
using namespace yarp::os;
using namespace yarp::sig;
using Pose = Eigen::Transform<double, 3, Eigen::Affine>;


bool Module::configure(yarp::os::ResourceFinder& rf)
{
    /* Get parameters. */
    const std::string robot = rf.check("robot", Value("icub")).asString();
    frequency_ = rf.check("frequency", Value(30)).asInt();

    const Bottle rf_gaze_limits = rf.findGroup("GAZE_LIMITS");
    enable_gaze_limit_x_ = rf_gaze_limits.check("enable_limit_x", Value(true)).asBool();
    enable_gaze_limit_y_ = rf_gaze_limits.check("enable_limit_y", Value(false)).asBool();
    enable_gaze_limit_z_ = rf_gaze_limits.check("enable_limit_z", Value(false)).asBool();
    gaze_limit_x_ = rf_gaze_limits.check("limit_x", Value(0.9)).asDouble();
    gaze_limit_y_ = rf_gaze_limits.check("limit_y", Value(0.0)).asDouble();
    gaze_limit_z_ = rf_gaze_limits.check("limit_z", Value(0.0)).asDouble();

    /* Open RPC port and attach to respond handler. */
    if (!port_rpc_.open("/" + log_name_ + "/rpc:i"))
    {
        yError() << log_name_ + "::ctor. Error: cannot open input RPC port.";

        return false;
    }
    if (!(this->yarp().attachAsServer(port_rpc_)))
    {
        yError() << log_name_ + "::ctor. Error: cannot attach RPC port to the respond handler.";
        return false;
    }

    /* Open RPC clients. */
    if (!port_rpc_segm_.open("/" + log_name_ + "/segmentation/rpc:o"))
    {
        yError() << log_name_ + "::ctor. Error: cannot open output RPC port towards segmentation module.";

        return false;
    }

    if (!port_rpc_pose_est_.open("/" + log_name_ + "/pose/rpc:o"))
    {
        yError() << log_name_ + "::ctor. Error: cannot open output RPC port towards pose estimation module.";

        return false;
    }

    if (!port_rpc_trk_.open("/" + log_name_ + "/tracker/rpc:o"))
    {
        yError() << log_name_ + "::ctor. Error: cannot open output RPC port towards pose tracker module.";

        return false;
    }

    /* Open port for object pose. */
    if (!port_pose_.open("/" + log_name_ + "/tracker/pose:i"))
    {
        yError() << log_name_ + "::ctor. Error: cannot open port for object pose.";

        return false;
    }

    /* Objects maps. */
    objects_map_["o003"] = "003_cracker_box";
    objects_map_["o004"] = "004_sugar_box";
    objects_map_["o006"] = "006_mustard_bottle";

    /* Configure iCub gaze controller. */
    gaze_ = std::make_unique<iCubGaze>(robot, log_name_);

    return true;
}


bool Module::close()
{
    port_rpc_.close();
    port_rpc_segm_.close();
    port_rpc_pose_est_.close();
    port_rpc_trk_.close();

    gaze_->go_home();
    gaze_->close();

    return true;
}


double Module::getPeriod()
{
    return (1.0 / frequency_);
}


bool Module::updateModule()
{
    /* Elapsed time from last received valid pose. */
    double elapsed = get_rx_elapsed_time();

    /* Get last pose if available. */
    bool valid_pose;
    Pose pose;
    std::tie(valid_pose, pose) = get_object_pose();

    /* Update reception time. */
    if (valid_pose)
        set_rx_time();

    if (state_ == State::Idle)
    {
        /* Restore idle gaze configuration. */
        gaze_->go_home();

        yInfo() << "[Idle -> WaitForFeedback]";
        state_ = State::WaitForFeedback;
    }
    else if (state_ == State::WaitForFeedback)
    {
        if (elapsed < feedback_wait_threshold_)
        {
            yInfo() << "[WaitForFeedback -> Tracking]";
            state_ = State::Tracking;
        }
    }
    else if (state_ == State::GoHome)
    {
        /* Restore idle gaze configuration. */
        gaze_->go_home();

        yInfo() << "[GoHome -> WaitForFeedback]";
        state_ = State::WaitForFeedback;
    }
    else if (state_ == State::Tracking)
    {
        if (elapsed > feedback_wait_threshold_)
        {
            yInfo() << "[Tracking -> GoHome]";
            state_ = State::GoHome;
        }
        else
        {
            /* Track object with gaze. */
            if (is_pose_gaze_safe())
            {
                Vector target(3);
                target(0) = last_object_pose_.translation()(0);
                target(1) = last_object_pose_.translation()(1);
                target(2) = last_object_pose_.translation()(2);
                gaze_->look_at_stream(target);
            }
        }
    }

    return true;
}


std::string Module::select_object(const std::string& object_name)
{
    bool found = false;
    for (const auto& pair : objects_map_)
    {
        if (object_name == pair.first)
        {
            found = true;
            break;
        }
    }
    if (!found)
    {
        std::string reply = "Available objects are ";
        std::size_t counter = 0;
        for (const auto& pair : objects_map_)
        {
            reply += pair.first + " -> " + pair.second;
            if (counter != (objects_map_.size() - 1))
                reply += ", ";

            counter++;
        }

        return reply;
    }

    send_rpc(port_rpc_segm_, {"select_object", objects_map_.at(object_name)});
    std::this_thread::sleep_for(100ms);

    send_rpc(port_rpc_pose_est_, {"select_object", objects_map_.at(object_name)});
    std::this_thread::sleep_for(100ms);

    send_rpc(port_rpc_trk_, {"select_object", objects_map_.at(object_name)});
    std::this_thread::sleep_for(100ms);
    send_rpc(port_rpc_trk_, {"stop"});
    std::this_thread::sleep_for(100ms);
    send_rpc(port_rpc_trk_, {"reset"});
    std::this_thread::sleep_for(100ms);
    send_rpc(port_rpc_trk_, {"start"});

    return "Command accepted";
}


std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> Module::get_object_pose()
{
    Vector* pose_yarp = port_pose_.read(is_pose_input_buffered_);

    auto yarp_to_transform = [](const Vector& vector) -> Pose
    {
        Pose pose;
        pose = Translation<double, 3>(vector[0], vector[1], vector[2]);
        pose.rotate(AngleAxisd(vector[6], Vector3d(vector[3], vector[4], vector[5])));

        return pose;
    };

    if (pose_yarp == nullptr)
    {
        if(is_pose_input_buffered_ && is_first_pose_)
            return std::make_pair(true, last_object_pose_);
        else
            return std::make_pair(false, Pose::Identity());
    }
    else
    {
        last_object_pose_ = yarp_to_transform(*pose_yarp);

        is_first_pose_ = true;

        return std::make_pair(true, last_object_pose_);
    }
}


bool Module::is_pose_gaze_safe()
{
    const double& x = last_object_pose_.translation()(0);
    const double& y = last_object_pose_.translation()(1);
    const double& z = last_object_pose_.translation()(2);

    if (enable_gaze_limit_x_ && (abs(x) > gaze_limit_x_))
        return false;

    if (enable_gaze_limit_y_ && (abs(y) > gaze_limit_y_))
        return false;

    if (enable_gaze_limit_z_ && (abs(z) > gaze_limit_z_))
        return false;

    return true;
}


double Module::get_rx_elapsed_time()
{
    auto now = std::chrono::steady_clock::now();

    if (is_first_time_)
    {
        input_delta_rx_ = 1000.0;
        last_rx_time_ = now;

        is_first_time_ = false;
    }
    else
    {
        double delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_time_).count();
        input_delta_rx_ = alpha_ema_ * delta + (1 - alpha_ema_) * input_delta_rx_;
    }

    if (input_delta_rx_ > input_delta_rx_max_)
        input_delta_rx_ = input_delta_rx_max_;

    return input_delta_rx_;
}


void Module::set_rx_time()
{
    last_rx_time_ = std::chrono::steady_clock::now();
}


bool Module::send_rpc(const yarp::os::RpcClient& port, std::vector<std::string> messages)
{
    Bottle cmd, reply;

    for (const std::string& message : messages)
        cmd.addString(message);

    return port.write(cmd, reply);
}
