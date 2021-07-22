/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef MODULE_H
#define MODULE_H

#include <Eigen/Dense>

#include <iCubGaze.h>

#include <thrift/ModuleIDL.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

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
    /**
     * Get object pose.
     */
    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> get_object_pose();

    /**
     * Send an RPC message.
     */
    bool send_rpc(const yarp::os::RpcClient& port, std::vector<std::string> messages);

    /**
     * RPC ports.
     */
    yarp::os::Port port_rpc_;

    yarp::os::RpcClient port_rpc_segm_;

    yarp::os::RpcClient port_rpc_pose_est_;

    yarp::os::RpcClient port_rpc_trk_;

    /**
     * Objects short name to long name mapping.
     */
    std::unordered_map<std::string, std::string> objects_map_;

    /**
     * iCub gaze controller.
     */
    std::unique_ptr<iCubGaze> gaze_;

    /**
     * Object pose input.
     */
    yarp::os::BufferedPort<yarp::sig::Vector> port_pose_;

    Eigen::Transform<double, 3, Eigen::Affine> last_object_pose_;

    bool is_pose_input_buffered_;

    bool is_first_pose_;

    /**
     * Parameters.
     */
    int frequency_;

    /**
     * Name for log messages.
     */
    const std::string log_name_ = "roft-track-n-grasp";
};

#endif /* MODULE_H */
