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

#include <chrono>
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
     * Get object state and validation.
     */
    std::tuple<bool, Eigen::Transform<double, 3, Eigen::Affine>, Eigen::Vector3d> get_object_state();

    bool is_pose_gaze_safe(const Eigen::Transform<double, 3, Eigen::Affine>& pose);

    bool is_object_steady(const Eigen::Vector3d& velocity);

    /**
     * Get/set reception time and elapsed time.
     */
    double get_rx_elapsed_time();

    void set_rx_time();

    /**
     * Send an RPC message.
     */
    bool send_rpc(const yarp::os::RpcClient& port, std::vector<std::string> messages);

    /**
     * iCub gaze controller.
     */
    std::unique_ptr<iCubGaze> gaze_;

    bool enable_gaze_limit_x_;
    bool enable_gaze_limit_y_;
    bool enable_gaze_limit_z_;

    double gaze_limit_x_;
    double gaze_limit_y_;
    double gaze_limit_z_;

    /**
     * Objects short name to long name mapping.
     */
    std::unordered_map<std::string, std::string> objects_map_;

    /**
     * Object pose input.
     */
    yarp::os::BufferedPort<yarp::sig::Vector> port_state_;

    Eigen::Transform<double, 3, Eigen::Affine> last_object_pose_;

    Eigen::Vector3d last_object_velocity_;

    const bool is_pose_input_buffered_ = false;

    bool is_first_state_ = false;

    /**
     * Object input reception time.
     */
    double alpha_ema_ = 0.1;

    double input_delta_rx_;

    const double input_delta_rx_max_ = 3000;

    std::chrono::steady_clock::time_point last_rx_time_;

    bool is_first_time_ = true;

    /**
     * Object steady state detector.
     */
    bool obj_ss_timer_init_ = false;

    double obj_ss_velocity_thr_;

    double obj_ss_time_thr_;

    std::chrono::steady_clock::time_point obj_ss_start_time_;

    /**
     * RPC ports.
     */
    yarp::os::Port port_rpc_;

    yarp::os::RpcClient port_rpc_segm_;

    yarp::os::RpcClient port_rpc_pose_est_;

    yarp::os::RpcClient port_rpc_trk_;

    /**
     * Parameters.
     */
    int frequency_;

    const double feedback_wait_threshold_ = 40.0;

    /**
     * Module state.
     */
    enum class State { Idle, WaitForFeedback, GoHome, Tracking };

    State state_ = State::Idle;

    /**
     * Name for log messages.
     */
    const std::string log_name_ = "roft-track-n-grasp";
};

#endif /* MODULE_H */
