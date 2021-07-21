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

using namespace std::literals::chrono_literals;
using namespace yarp::os;



bool Module::configure(yarp::os::ResourceFinder& rf)
{
    /* Get parameters. */
    const std::string robot = rf.check("robot", Value("icub")).asString();

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
    gaze_->close();

    return true;
}


double Module::getPeriod()
{
    return (1.0 / 30.0);
}


bool Module::updateModule()
{
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


bool Module::send_rpc(const yarp::os::RpcClient& port, std::vector<std::string> messages)
{
    Bottle cmd, reply;

    for (const std::string& message : messages)
        cmd.addString(message);

    return port.write(cmd, reply);
}
