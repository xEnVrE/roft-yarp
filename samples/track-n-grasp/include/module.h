/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef MODULE_H
#define MODULE_H

#include <thrift/ModuleIDL.h>

#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RFModule.h>

#include <string>
#include <unordered_map>


class Module : public yarp::os::RFModule,
               public ModuleIDL
{
public:
    bool configure(yarp::os::ResourceFinder& rf) override;

    bool close() override;

    double getPeriod() override;

    bool updateModule() override;

    /**
     * IDL interface.
     */

    std::string select_object(const std::string& object_name) override;

private:
    /* Send an RPC message. */
    bool send_rpc(const yarp::os::RpcClient& port, std::vector<std::string> messages);

    /* RPC input port. */
    yarp::os::Port port_rpc_;

    /* RPC clients. */
    yarp::os::RpcClient port_rpc_segm_;

    yarp::os::RpcClient port_rpc_pose_est_;

    yarp::os::RpcClient port_rpc_trk_;

    /* Objects map. */
    std::unordered_map<std::string, std::string> objects_map_;

    /* Log name for messages. */
    const std::string log_name_ = "roft-track-n-grasp";
};

#endif /* MODULE_H */
