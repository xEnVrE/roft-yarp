/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <module.h>
#include <cardinal_points_grasp.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>

#include <thread>

using namespace Eigen;
using namespace Utils;
using namespace cardinal_points_grasp;
using namespace std::literals::chrono_literals;
using namespace yarp::eigen;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using Pose = Eigen::Transform<double, 3, Eigen::Affine>;


bool Module::configure(yarp::os::ResourceFinder& rf)
{
    /* Get parameters. */
    const std::string robot = rf.check("robot", Value("icub")).asString();
    frequency_ = rf.check("frequency", Value(30)).asInt();

    const Bottle rf_cartesian_control = rf.findGroup("CARTESIAN_CONTROL");
    approach_traj_time_ = rf_cartesian_control.check("approach_traj_time", Value(5.0)).asDouble();
    double torso_pitch_max = rf_cartesian_control.check("torso_pitch_max", Value(10.0)).asDouble();
    double torso_pitch_min = rf_cartesian_control.check("torso_pitch_min", Value(-10.0)).asDouble();

    const Bottle rf_gaze_limits = rf.findGroup("GAZE_LIMITS");
    enable_gaze_limit_x_ = rf_gaze_limits.check("enable_limit_x", Value(true)).asBool();
    enable_gaze_limit_y_ = rf_gaze_limits.check("enable_limit_y", Value(false)).asBool();
    enable_gaze_limit_z_ = rf_gaze_limits.check("enable_limit_z", Value(false)).asBool();
    gaze_limit_x_ = rf_gaze_limits.check("limit_x", Value(0.9)).asDouble();
    gaze_limit_y_ = rf_gaze_limits.check("limit_y", Value(0.0)).asDouble();
    gaze_limit_z_ = rf_gaze_limits.check("limit_z", Value(0.0)).asDouble();

    const Bottle rf_grasp_limits = rf.findGroup("GRASP_LIMITS");
    grasp_limit_x_lower_ = rf_grasp_limits.check("limit_x_lower", Value(-0.4)).asDouble();
    grasp_limit_x_upper_ = rf_grasp_limits.check("limit_x_upper", Value(-0.2)).asDouble();
    grasp_limit_y_lower_ = rf_grasp_limits.check("limit_y_lower", Value(-0.2)).asDouble();
    grasp_limit_y_upper_ = rf_grasp_limits.check("limit_y_upper", Value(0.2)).asDouble();
    grasp_limit_z_lower_ = rf_grasp_limits.check("limit_z_lower", Value(0.0)).asDouble();
    grasp_limit_z_upper_ = rf_grasp_limits.check("limit_z_upper", Value(0.3)).asDouble();

    const Bottle rf_joint_control = rf.findGroup("JOINT_CONTROL");
    bool is_vector;
    Vector arm_joint_home_configuration;
    std::tie(is_vector, arm_joint_home_configuration) = load_vector_double(rf_joint_control, "home_configuration", 7);
    if (!is_vector)
    {
        yError() << log_name_ + "::configure(). Error: cannot get parameter JOINT_CONTROL::home_configuration";
        return false;
    }

    const Bottle rf_parts = rf.findGroup("PARTS");
    bool enable_part_left = rf_parts.check("left", Value(false)).asBool();
    bool enable_part_right = rf_parts.check("right", Value(false)).asBool();

    const Bottle rf_steady_state_detector = rf.findGroup("STEADY_STATE_DETECTOR");
    obj_ss_time_thr_ = rf_steady_state_detector.check("time_threshold", Value(4.0)).asDouble();
    obj_ss_velocity_thr_ = rf_steady_state_detector.check("vel_threshold", Value(0.05)).asDouble();

    /* Open RPC port and attach to respond handler. */
    if (!port_rpc_.open("/" + log_name_ + "/rpc:i"))
    {
        yError() << log_name_ + "::configure. Error: cannot open input RPC port.";

        return false;
    }
    if (!(this->yarp().attachAsServer(port_rpc_)))
    {
        yError() << log_name_ + "::configure. Error: cannot attach RPC port to the respond handler.";
        return false;
    }

    /* Open RPC clients. */
    if (!port_rpc_segm_.open("/" + log_name_ + "/segmentation/rpc:o"))
    {
        yError() << log_name_ + "::configure. Error: cannot open output RPC port towards segmentation module.";

        return false;
    }

    if (!port_rpc_pose_est_.open("/" + log_name_ + "/pose/rpc:o"))
    {
        yError() << log_name_ + "::configure. Error: cannot open output RPC port towards pose estimation module.";

        return false;
    }

    if (!port_rpc_trk_.open("/" + log_name_ + "/tracker/rpc:o"))
    {
        yError() << log_name_ + "::configure. Error: cannot open output RPC port towards pose tracker module.";

        return false;
    }

    /* Open port for object state. */
    if (!port_state_.open("/" + log_name_ + "/tracker/state:i"))
    {
        yError() << log_name_ + "::configure. Error: cannot open port for object state.";

        return false;
    }

    /* Objects maps. */
    objects_map_["o003"] = "003_cracker_box";
    objects_map_["o004"] = "004_sugar_box";
    objects_map_["o006"] = "006_mustard_bottle";

    /* Configure iCub gaze controller. */
    gaze_ = std::make_shared<iCubGaze>(robot, log_name_);

    /* Configure iCub joint controllers. */
    joints_left_arm_ = std::make_shared<iCubMotorsPositions>(robot, "left", log_name_ + "/left_arm", /* use_arm = */ true, /* use_torso = */ false);
    joints_right_arm_ = std::make_shared<iCubMotorsPositions>(robot, "right", log_name_ + "/right_arm", /* use_arm = */ true, /* use_torso = */ false);
    joints_torso_ = std::make_shared<iCubMotorsPositions>(robot, "no_laterality", log_name_ + "/torso", /* use_arm = */ false, /* use_torso = */ true);
    joints_left_hand_ = std::make_shared<iCubMotorsPositions>(robot, "left", log_name_ + "/left_hand", /* use_arm = */ true, /* use_torso = */ false);
    joints_right_hand_ = std::make_shared<iCubMotorsPositions>(robot, "right", log_name_ + "/right_hand", /* use_arm = */ true, /* use_torso = */ false);

    /* Configure iCub Cartesian controllers. */
    if (enable_part_left)
    {
        cart_left_ = std::make_shared<iCubCartesian>(robot, "left", log_name_);
        cart_left_->enable_torso(true, true, false);
        cart_left_->enable_torso_limits("pitch", torso_pitch_min, torso_pitch_max);
    }
    if (enable_part_right)
    {
        cart_right_ = std::make_shared<iCubCartesian>(robot, "right", log_name_);
        cart_right_->enable_torso(true, true, false);
        cart_right_->enable_torso_limits("pitch", torso_pitch_min, torso_pitch_max);
    }

    /* Set joints for home configuration. */
    home_torso_joints_ = Vector3d::Zero();
    home_arm_joints_ = toEigen(arm_joint_home_configuration);
    home_hand_joints_ = VectorXd::Zero(9);
    /* iCub joint 'hand_finger' */
    home_hand_joints_(0) = 45.0;
    /* iCub joint 'thumb_opposition' */
    home_hand_joints_(1) = 10.0;
    /* Set hand joints for pregrasp configuration. */
    pregrasp_hand_joints_ = VectorXd::Zero(9);
    pregrasp_hand_joints_(0) = 45.0;
    pregrasp_hand_joints_(1) = 80.0;
    /* Set hand joints for grasp configuration. */
    grasp_hand_joints_ = VectorXd::Zero(9);
    grasp_hand_joints_(0) = 45.0;
    grasp_hand_joints_(1) = 80.0;
    grasp_hand_joints_(2) = 40.0;
    grasp_hand_joints_(3) = 35.0;
    grasp_hand_joints_(4) = 60.0;
    grasp_hand_joints_(5) = 40.0;
    grasp_hand_joints_(6) = 60.0;
    grasp_hand_joints_(7) = 40.0;
    grasp_hand_joints_(8) = 100.0;
    /* Set hand joints for post grasp configuration. */
    postgrasp_hand_joints_ = home_hand_joints_;
    postgrasp_hand_joints_(1) = 80.0;

    /* Set joints velocities for arm home configuration to 5.0 deg/s. */
    home_torso_joints_vels_ = VectorXd::Ones(home_torso_joints_.size()) * 10.0;
    home_arm_joints_vels_ = VectorXd::Ones(home_arm_joints_.size()) * 10.0;
    /* Set joints velocities for hand home configuration to 100.0 deg/s. */
    home_hand_joints_vels_ = VectorXd::Ones(home_hand_joints_.size()) * 100.0;
    home_hand_joints_vels_(1) = 50.0;
    /* Set hand joints velocities for pregrasp. */
    pregrasp_hand_joints_vels_ = VectorXd::Ones(pregrasp_hand_joints_.size()) * 100.0;
    grasp_hand_joints_vels_ = VectorXd::Ones(pregrasp_hand_joints_.size()) * 100.0;

    return true;
}


bool Module::close()
{
    if (cart_left_)
        cart_left_->stop();

    if (cart_right_)
        cart_right_->stop();

    if (joints_left_arm_)
        joints_left_arm_->stop(home_arm_considered_joints_);

    if (joints_right_arm_)
        joints_right_arm_->stop(home_arm_considered_joints_);

    if (joints_torso_)
        joints_torso_->stop(home_torso_considered_joints_);

    if (gaze_)
        gaze_->stop();

    if (joints_left_hand_)
        joints_left_hand_->stop(home_hand_considered_joints_);

    if (joints_right_hand_)
        joints_right_hand_->stop(home_hand_considered_joints_);;

    if (cart_left_)
        cart_left_->close();

    if (cart_right_)
        cart_right_->close();

    if (joints_left_arm_)
        joints_left_arm_->close();

    if (joints_right_arm_)
        joints_right_arm_->close();

    if (joints_torso_)
        joints_torso_->close();

    if (gaze_)
        gaze_->close();

    if (joints_left_hand_)
        joints_left_hand_->close();

    if (joints_right_hand_)
        joints_right_hand_->close();

    port_rpc_.close();
    port_rpc_segm_.close();
    port_rpc_pose_est_.close();
    port_rpc_trk_.close();

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
    Vector3d velocity;
    std::tie(valid_pose, pose, velocity) = get_object_state();

    /* Update reception time. */
    if (valid_pose)
        set_rx_time();

    if (state_ == State::Idle)
    {
        /* Restore gaze idle configuration. */
        gaze_->go_home();

        /* Restore torso and arms idle configuration. */
        go_home_arm();

        /* Restore hand idle configuration. */
        go_home_hand();

        start_counting(3.0);

        yInfo() << "[Idle -> WaitForHome]";
        state_ = State::WaitForHome;
    }
    else if (state_ == State::GoHome)
    {
        /* Restore idle gaze configuration. */
        gaze_->go_home();

        yInfo() << "[GoHome -> WaitForFeedback]";
        state_ = State::WaitForFeedback;
    }
    else if (state_ == State::Grasp)
    {
        if (grasp_state_ == GraspState::Done)
        {
            yInfo() << "[Grasp][Done -> Idle]";

            grasp_state_ = GraspState::Idle;

            yInfo() << "[Grasp -> Idle]";

            state_ = State::Idle;
        }

        if(!execute_grasp(grasp_object_pose_))
        {
            yInfo() << "[Grasp -> Idle]";

            state_ = State::Idle;
        }
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
            if (valid_pose && is_pose_gaze_safe(pose))
            {
                Vector target(3);
                target(0) = pose.translation()(0);
                target(1) = pose.translation()(1);
                target(2) = pose.translation()(2);
                gaze_->look_at_stream(target);
            }

            if (is_object_steady(velocity) && is_pose_grasp_safe(pose))
            {
                grasp_object_pose_ = pose;
                grasp_state_ = GraspState::Evaluation;

                yInfo() << "[Tracking -> Grasp]";
                state_ = State::Grasp;
            }
        }
    }
    else if (state_ == State::WaitForFeedback)
    {
        if (elapsed < feedback_wait_threshold_)
        {
            yInfo() << "[WaitForFeedback -> Tracking]";
            state_ = State::Tracking;
        }
    }
    else if (state_ == State::WaitForHome)
    {
        if (is_elapsed_from_start_counting())
        {
            yInfo() << "[WaitForHome -> WaitForFeedback]";
            state_ = State::WaitForFeedback;
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

    object_name_ = object_name;

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


std::tuple<bool, Eigen::Transform<double, 3, Eigen::Affine>, Vector3d> Module::get_object_state()
{
    Vector* state_yarp = port_state_.read(is_pose_input_buffered_);

    auto yarp_to_transform = [](const Vector& vector) -> Pose
    {
        Pose pose;
        pose = Translation<double, 3>(vector[0], vector[1], vector[2]);
        pose.rotate(AngleAxisd(vector[6], Vector3d(vector[3], vector[4], vector[5])));

        return pose;
    };

    auto yarp_to_velocity = [](const Vector& vector) -> Vector3d
    {
        Vector3d velocity(vector[7], vector[8], vector[9]);

        return velocity;
    };

    if (state_yarp == nullptr)
    {
        if(is_pose_input_buffered_ && is_first_state_)
            return std::make_tuple(true, last_object_pose_, last_object_velocity_);
        else
            return std::make_tuple(false, Pose::Identity(), Vector3d::Zero());
    }
    else
    {
        last_object_pose_ = yarp_to_transform(*state_yarp);
        last_object_velocity_ = yarp_to_velocity(*state_yarp);

        is_first_state_ = true;

        return std::make_tuple(true, last_object_pose_, last_object_velocity_);
    }
}


bool Module::is_pose_gaze_safe(const Pose& pose)
{
    const double& x = pose.translation()(0);
    const double& y = pose.translation()(1);
    const double& z = pose.translation()(2);

    if (enable_gaze_limit_x_ && (abs(x) > gaze_limit_x_))
        return false;

    if (enable_gaze_limit_y_ && (abs(y) > gaze_limit_y_))
        return false;

    if (enable_gaze_limit_z_ && (abs(z) > gaze_limit_z_))
        return false;

    return true;
}


bool Module::is_pose_grasp_safe(const Pose& pose)
{
    const double& x = pose.translation()(0);
    const double& y = pose.translation()(1);
    const double& z = pose.translation()(2);

    if (x > grasp_limit_x_upper_)
        return false;

    if (x < grasp_limit_x_lower_)
        return false;

    if (y > grasp_limit_y_upper_)
        return false;

    if (y < grasp_limit_y_lower_)
        return false;

    if (z > grasp_limit_z_upper_)
        return false;

    if (z < grasp_limit_z_lower_)
        return false;

    return true;
}


bool Module::is_position_cart_safe(const yarp::sig::Vector& position)
{
    Pose pose;
    pose = Translation<double, 3>(position[0], position[1], position[2]);

    return is_pose_grasp_safe(pose);
}


bool Module::is_object_steady(const Eigen::Vector3d& velocity)
{
    /* Check if object is steady state. */
    if (velocity.norm() < obj_ss_velocity_thr_)
    {
        if (obj_ss_timer_init_)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - obj_ss_start_time_).count() / 1000.0;

            if (elapsed > obj_ss_time_thr_)
                return true;
        }
        else
        {
            obj_ss_start_time_ = std::chrono::steady_clock::now();

            obj_ss_timer_init_ = true;
        }
    }
    else
        obj_ss_timer_init_ = false;

    return false;
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


void Module::start_counting(const double& total_time)
{
    time_0_ = std::chrono::steady_clock::now();

    total_time_ = total_time;
}


bool Module::is_elapsed_from_start_counting()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_0_).count() / 1000.0 > total_time_;
}


bool Module::send_rpc(const yarp::os::RpcClient& port, std::vector<std::string> messages)
{
    Bottle cmd, reply;

    for (const std::string& message : messages)
        cmd.addString(message);

    return port.write(cmd, reply);
}


void Module::go_home_arm()
{
    /* Switch to POSITION control. */
    if (joints_torso_)
        joints_torso_->set_mode(home_torso_considered_joints_);
    if (joints_left_arm_)
        joints_left_arm_->set_mode(home_arm_considered_joints_);
    if (joints_right_arm_)
        joints_right_arm_->set_mode(home_arm_considered_joints_);

    /* Send setpoint. */
    if (joints_torso_)
        joints_torso_->set_positions(home_torso_joints_, home_torso_joints_vels_, home_torso_considered_joints_);
    if (joints_left_arm_)
        joints_left_arm_->set_positions(home_arm_joints_, home_arm_joints_vels_, home_arm_considered_joints_);
    if (joints_right_arm_)
        joints_right_arm_->set_positions(home_arm_joints_, home_arm_joints_vels_, home_arm_considered_joints_);
}

void Module::go_home_hand()
{
    /* Switch to POSITION control. */
    if (joints_left_hand_)
        joints_left_hand_->set_mode(home_hand_considered_joints_);
    if (joints_right_hand_)
        joints_right_hand_->set_mode(home_hand_considered_joints_);

    /* Send setpoint. */
    if (joints_left_hand_)
        joints_left_hand_->set_positions(home_hand_joints_, home_hand_joints_vels_, home_hand_considered_joints_);
    if (joints_right_hand_)
        joints_right_hand_->set_positions(home_hand_joints_, home_hand_joints_vels_, home_hand_considered_joints_);
}


bool Module::execute_grasp(const Pose& pose)
{
    if (grasp_state_ == GraspState::Evaluation)
    {
        /* Keep gazing at the object. */
        gaze_->controller().setTrackingMode(true);
        Vector target(3);
        target(0) = pose.translation()(0);
        target(1) = pose.translation()(1);
        target(2) = pose.translation()(2);
        gaze_->look_at_stream(target);

        /* Cardinal points grasp strategy. */

        if ((cart_left_ == nullptr) && (cart_right_ == nullptr))
        {
            gaze_->stop();
            gaze_->controller().setTrackingMode(false);

            return false;
        }

        std::vector<cardinal_points_grasp::rankable_candidate> candidates;
        std::vector<cardinal_points_grasp::rankable_candidate> candidates_l;
        std::vector<cardinal_points_grasp::rankable_candidate> candidates_r;
        int context_l;
        int context_r;

        if (cart_left_)
        {
            auto grasper = std::make_unique<CardinalPointsGrasp>(object_name_, "left", pregrasp_hand_joints_);
            std::tie(candidates_l, context_l) = grasper->getCandidates(pose, &(cart_left_->controller()));
        }

        if (cart_right_)
        {
            auto grasper = std::make_unique<CardinalPointsGrasp>(object_name_, "right", pregrasp_hand_joints_);
            std::tie(candidates_r, context_r) = grasper->getCandidates(pose, &(cart_right_->controller()));
        }

        if (cart_right_ && (cart_left_ == nullptr))
            candidates = candidates_r;
        else if(cart_left_ && (cart_right_ == nullptr))
            candidates = candidates_l;
        else
        {
            candidates = candidates_r;
            candidates.insert(candidates.end(), candidates_l.begin(), candidates_l.end());
            std::sort(candidates.begin(), candidates.end(), CardinalPointsGrasp::compareCandidates);
        }

        if (candidates.empty())
        {
            gaze_->stop();
            gaze_->controller().setTrackingMode(false);

            yError() << log_name_ << "::execute_grasp(). No valid grasping candidates.";

            return false;
        }

        const auto& best = candidates[0];
        grasp_type_ = std::get<0>(best);
        const auto& T = std::get<2>(best);
        grasp_center_ = std::get<3>(best);
        grasp_target_position_ = T.getCol(3).subVector(0, 2);
        grasp_target_orientation_ = dcm2axis(T);

        yInfo() << log_name_ + "::execute_grasp(). Found grasp pose using" << grasp_type_ << "arm at" << grasp_target_position_.toString();

        /* Arm selection. */

        int context;
        std::shared_ptr<iCubCartesian> cart;
        std::shared_ptr<iCubMotorsPositions> joints_hand;
        if (grasp_type_ == "left")
        {
            grasp_context_ = context_l;
            grasp_cart_ = cart_left_;
            grasp_joints_hand_ = joints_left_hand_;
        }
        else
        {
            grasp_context_ = context_r;
            grasp_cart_ = cart_right_;
            grasp_joints_hand_ = joints_right_hand_;
        }

        if ((grasp_cart_ == nullptr) || (grasp_joints_hand_ == nullptr))
        {
            gaze_->stop();
            gaze_->controller().setTrackingMode(false);

            yError() << log_name_ << "::execute_grasp. Unexpected state. Either the cartesian or joints driver pointers are nullptr";

            return false;
        }

        /* Cartesian configuration .*/
        grasp_cart_->controller().stopControl();
        grasp_cart_->controller().storeContext(&backup_grasp_context_);
        grasp_cart_->controller().restoreContext(grasp_context_);
        grasp_cart_->controller().setInTargetTol(.001);
        grasp_cart_->controller().setTrajTime(approach_traj_time_);

        /* Hand pregrasp configuration .*/
        grasp_joints_hand_->set_positions(pregrasp_hand_joints_, pregrasp_hand_joints_vels_, hand_considered_joints_);

        yInfo() << "[Grasp][Evaluate -> WaitHandPregrasp]";

        grasp_state_ = GraspState::WaitHandPregrasp;
        start_counting(1.0);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitHandPregrasp)
    {
        if (is_elapsed_from_start_counting())
        {
            if (grasp_joints_hand_->check_motion_done(hand_considered_joints_))
            {
                yInfo() << log_name_ + "::execute_grasp(). Hand ready in pregrasp configuration";

                yInfo() << "[Grasp][Evaluate -> ArmPregrasp]";

                grasp_state_ = GraspState::ArmPregrasp;

                return true;
            }
            else
                start_counting(1.0);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::ArmPregrasp)
    {

        /* Arm pregrasp configuration. */
        const auto dir = grasp_target_position_ - grasp_center_;

        if (!is_position_cart_safe(grasp_target_position_ + .05 * dir / norm(dir)))
        {
            yError() << "[Grasp][ArmPregrasp] *********** The commanded position is not safe ***********";

            yInfo() << "[Grasp][ArmPregrasp -> Cleanup]";

            grasp_state_ = GraspState::Cleanup;

            return true;
        }

        grasp_cart_->go_to_pose(grasp_target_position_ + .05 * dir / norm(dir), grasp_target_orientation_);

        yInfo() << "[Grasp][ArmPregrasp -> WaitArmPregrasp]";

        grasp_state_ = GraspState::WaitArmPregrasp;
        // start_counting(0.5);
        start_counting(5.0);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitArmPregrasp)
    {
        if (is_elapsed_from_start_counting())
        {
            // if(grasp_cart_->check_motion_done())
            // {
                yInfo() << log_name_ + "::execute_grasp(). " + grasp_type_ << "arm ready in pregrasp configuration";

                yInfo() << "[Grasp][WaitArmPregrasp -> ArmReach]";

                grasp_state_ = GraspState::ArmReach;
            // }
            // else
            //     start_counting(0.5);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::ArmReach)
    {
        /* Arm final configuration. */

        if (!is_position_cart_safe(grasp_target_position_))
        {
            yError() << "[Grasp][ArmReach] *********** The commanded position is not safe ***********";

            yInfo() << "[Grasp][ArmReach -> Cleanup]";

            grasp_state_ = GraspState::Cleanup;

            return true;
        }

        grasp_cart_->go_to_pose(grasp_target_position_, grasp_target_orientation_);

        yInfo() << "[Grasp][ArmReach -> WaitArmReach]";

        grasp_state_ = GraspState::WaitArmReach;
        // start_counting(0.5);
        start_counting(5.0);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitArmReach)
    {
        if (is_elapsed_from_start_counting())
        {
            // if(grasp_cart_->check_motion_done())
            // {
                yInfo() << log_name_ + "::execute_grasp(). " + grasp_type_ << "arm ready in grasping configuration";

                yInfo() << "[Grasp][WaitArmReach -> Grasp]";

                grasp_state_ = GraspState::Grasp;
            // }
            // else
            //     start_counting(0.5);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::Grasp)
    {
        /* Hand grasp configuration .*/
        grasp_joints_hand_->set_positions(grasp_hand_joints_, grasp_hand_joints_vels_, hand_considered_joints_);

        yInfo() << "[Grasp][Grasp -> WaitGrasp]";

        grasp_state_ = GraspState::WaitGrasp;
        start_counting(6.0);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitGrasp)
    {
        if (is_elapsed_from_start_counting())
        {
            if (grasp_joints_hand_->check_motion_done(hand_considered_joints_))
            {
                yInfo() << log_name_ + "::execute_grasp()." << grasp_type_ << "hand ready in grasping configuration";

                yInfo() << "[Grasp][WaitGrasp -> Lift]";

                grasp_state_ = GraspState::Lift;

                return true;
            }
            else
                start_counting(1.0);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::Lift)
    {
        /* Object lifting .*/
        yarp::sig::Vector target_up = grasp_target_position_ + yarp::sig::Vector{0.0, 0.0, 0.1};
        gaze_->look_at_stream(target_up);

        if (!is_position_cart_safe(target_up))
        {
            yError() << "[Grasp][Lift] *********** The commanded position is not safe ***********";

            yInfo() << "[Grasp][Lift -> Cleanup]";

            grasp_state_ = GraspState::Cleanup;

            return true;
        }

        grasp_cart_->go_to_pose(target_up, grasp_target_orientation_);

        yInfo() << "[Grasp][Lift -> WaitLift]";

        grasp_state_ = GraspState::WaitLift;
        // start_counting(0.5);
        start_counting(5.0);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitLift)
    {
        if (is_elapsed_from_start_counting())
        {
            // if(grasp_cart_->check_motion_done())
            // {
                yInfo() << log_name_ + "::execute_grasp(). Lifted completed.";

                yInfo() << "[Grasp][WaitLift -> WaitAfterLift]";

                grasp_state_ = GraspState::WaitAfterLift;

                start_counting(3.0);
            // }
            // else
            //     start_counting(0.5);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::WaitAfterLift)
    {
        if (is_elapsed_from_start_counting())
        {
            yInfo() << log_name_ + "::execute_grasp(). Wait after lift completed.";

            yInfo() << "[Grasp][WaitAfterLift -> Release]";

            grasp_state_ = GraspState::Release;
        }

        return true;
    }
    else if (grasp_state_ == GraspState::Release)
    {
        /* Object release .*/
        grasp_joints_hand_->set_positions(postgrasp_hand_joints_, grasp_hand_joints_vels_, hand_considered_joints_);
        yInfo() << "[Grasp][Release -> WaitRelease]";

        grasp_state_ = GraspState::WaitRelease;
        start_counting(3.0);

        return true;
    }
    else if (grasp_state_ == GraspState::WaitRelease)
    {
        if (is_elapsed_from_start_counting())
        {
            if (grasp_joints_hand_->check_motion_done(hand_considered_joints_))
            {
                yInfo() << log_name_ + "::execute_grasp(). Release done.";

                yInfo() << "[Grasp][WaitRelease -> Cleanup]";

                grasp_state_ = GraspState::Cleanup;

                return true;
            }
            else
                start_counting(1.0);
        }

        return true;
    }
    else if (grasp_state_ == GraspState::Cleanup)
    {
        yInfo() << log_name_ + "::execute_grasp(). Cleaning up.";

        gaze_->stop();
        gaze_->controller().setTrackingMode(false);

        grasp_cart_->stop();
        grasp_cart_->controller().restoreContext(backup_grasp_context_);

        grasp_cart_ = nullptr;
        grasp_joints_hand_ = nullptr;

        yInfo() << "[Grasp][Cleanup -> Done]";

        grasp_state_ = GraspState::Done;

        return true;
    }

    return true;
}
