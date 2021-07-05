/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <tracker.h>

#include <iostream>
#include <memory>

#include <OTL/ModelParameters.h>
#include <OTL/ImageOpticalFlowSource.h>
#include <OTL/ImageOpticalFlowNVOF.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/RealsenseCameraYarp.h>
#include <RobotsIO/Utils/Segmentation.h>
#include <RobotsIO/Utils/SegmentationYarpPort.h>
#include <RobotsIO/Utils/Transform.h>
#include <RobotsIO/Utils/TransformYarpPort.h>

using namespace Eigen;
using namespace OTL;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace yarp::os;


Tracker::Tracker(const ResourceFinder& rf)
{
    /* Sample time. */

    const double sample_time = rf.check("sample_time", Value(1.0 / 30.0)).asDouble();

    /* Camera. */

    const Bottle rf_camera = rf.findGroup("CAMERA");
    const std::string camera_source = rf_camera.check("source", Value("")).asString();
    const std::size_t camera_width = rf_camera.check("width", Value(1280)).asInt();
    const std::size_t camera_height = rf_camera.check("height", Value(720)).asInt();
    const double camera_fx = rf_camera.check("fx", Value(1230.0)).asDouble();
    const double camera_fy = rf_camera.check("fy", Value(1230.0)).asDouble();
    const double camera_cx = rf_camera.check("cx", Value(1280 / 2.0)).asDouble();
    const double camera_cy = rf_camera.check("cy", Value(720 / 2.0)).asDouble();

    /* Initial condition. */

    const Bottle rf_initial_conditions = rf.findGroup("INITIAL_CONDITION");

    const VectorXd p_v_0 = load_vector_double(rf_initial_conditions, "p_v_0", 3);
    const VectorXd p_w_0 = load_vector_double(rf_initial_conditions, "p_w_0", 3);

    const VectorXd p_cov_v_0 = load_vector_double(rf_initial_conditions, "p_cov_v_0", 3);
    const VectorXd p_cov_w_0 = load_vector_double(rf_initial_conditions, "p_cov_w_0", 3);
    const VectorXd p_cov_x_0 = load_vector_double(rf_initial_conditions, "p_cov_x_0", 3);
    const VectorXd p_cov_q_0 = load_vector_double(rf_initial_conditions, "p_cov_q_0", 3);

    const VectorXd v_v_0 = load_vector_double(rf_initial_conditions, "v_v_0", 3);
    const VectorXd v_w_0 = load_vector_double(rf_initial_conditions, "v_w_0", 3);
    const VectorXd v_cov_v_0 = load_vector_double(rf_initial_conditions, "v_cov_v_0", 3);
    const VectorXd v_cov_w_0 = load_vector_double(rf_initial_conditions, "v_cov_w_0", 3);

    /* Kinematic model. */

    const Bottle rf_kinematic_model = rf.findGroup("KINEMATIC_MODEL");

    const VectorXd psd_lin_acc = load_vector_double(rf_kinematic_model, "psd_lin_acc", 3);
    const VectorXd sigma_ang_vel = load_vector_double(rf_kinematic_model, "sigma_ang_vel", 3);

    const VectorXd kin_q_v = load_vector_double(rf_kinematic_model, "q_v", 3);
    const VectorXd kin_q_w = load_vector_double(rf_kinematic_model, "q_w", 3);

    /* Measurement model. */

    const Bottle rf_measurement = rf.findGroup("MEASUREMENT_MODEL");
    const VectorXd v_meas_cov_flow = load_vector_double(rf_measurement, "v_cov_flow", 2);

    const VectorXd p_meas_cov_v = load_vector_double(rf_measurement, "p_cov_v", 3);
    const VectorXd p_meas_cov_w = load_vector_double(rf_measurement, "p_cov_w", 3);
    const VectorXd p_meas_cov_x = load_vector_double(rf_measurement, "p_cov_x", 3);
    const VectorXd p_meas_cov_q = load_vector_double(rf_measurement, "p_cov_q", 3);

    const bool use_pose_measurement = rf_measurement.check("use_pose_measurement", Value(false)).asBool();
    const bool use_pose_resync = rf_measurement.check("use_pose_resync", Value(false)).asBool();
    const bool use_velocity_measurement = rf_measurement.check("use_vel_measurement", Value(false)).asBool();

    const double depth_maximum = rf_measurement.check("depth_maximum", Value(2.0)).asDouble();
    const double subsampling_radius = rf_measurement.check("subsampling_radius", Value(1.0)).asDouble();
    const bool flow_weighting = rf_measurement.check("flow_weighting", Value(false)).asBool();

    /* Model. */

    const Bottle rf_model = rf.findGroup("MODEL");
    const std::string model_name = rf_model.check("name", Value("")).asString();
    const bool model_use_internal_db = rf_model.check("use_internal_db", Value(true)).asBool();
    const std::string model_internal_db_name = rf_model.check("internal_db_name", Value("YCBVideo")).asString();
    const std::string model_external_path = rf_model.check("external_path", Value("")).asString();

    /* Optical flow. */

    const Bottle rf_optical_flow = rf.findGroup("OPTICAL_FLOW");
    const std::string optical_flow_source = rf_optical_flow.check("source", Value("")).asString();

    /* Outlier rejection. */

    const Bottle rf_outlier = rf.findGroup("OUTLIER_REJECTION");
    const bool outlier_rejection_enable = rf_outlier.check("enable", Value(false)).asBool();
    const double outlier_rejection_gain = rf_outlier.check("gain", Value(1.0)).asDouble();

    /* Pose .*/

    const Bottle& pose_bottle = rf.findGroup("POSE");
    const std::string pose_source = pose_bottle.check("source", Value("YARP")).asString();

   /* Segmentation. */
    const Bottle rf_segmentation = rf.findGroup("SEGMENTATION");
    const std::string segmentation_source = rf_segmentation.check("source", Value("")).asString();
    const bool flow_aided_segmentation = rf_segmentation.check("flow_aided", Value(false)).asBool();

    /* Unscented transform. */
    const Bottle rf_unscented_transform = rf.findGroup("UNSCENTED_TRANSFORM");
    const double ut_alpha = rf_unscented_transform.check("alpha", Value("1.0")).asDouble();
    const double ut_beta = rf_unscented_transform.check("beta", Value("2.0")).asDouble();
    const double ut_kappa = rf_unscented_transform.check("kappa", Value("0.0")).asDouble();

    /* Summary. */

    std::cout << log_name_ << " parameters:" << std::endl << std::endl;

    std::cout << "sample_time: " << sample_time << std::endl << std::endl;

    std::cout << "Camera:" << std::endl;

    std::cout << "- source: " << camera_source << std::endl << std::endl;
    if (camera_source == "YARP")
    {
        std::cout << "- width: " << camera_width << std::endl;
        std::cout << "- height: " << camera_height << std::endl;
        std::cout << "- fx: " << camera_fx << std::endl;
        std::cout << "- fy: " << camera_fy << std::endl;
        std::cout << "- cx: " << camera_cx << std::endl;
        std::cout << "- cy: " << camera_cy << std::endl;
    }

    std::cout << "Initial conditions:" << std::endl;

    std::cout << "  - position:" << std::endl;
    std::cout << "    - v_0: " << p_v_0.transpose() << std::endl;
    std::cout << "    - w_0: " << p_w_0.transpose() << std::endl;
    std::cout << "    - cov_v_0: " << p_cov_v_0.transpose() << std::endl;
    std::cout << "    - cov_w_0: " << p_cov_w_0.transpose() << std::endl;
    std::cout << "    - cov_x_0: " << p_cov_x_0.transpose() << std::endl;
    std::cout << "    - cov_q_0: " << p_cov_q_0.transpose() << std::endl << std::endl;

    std::cout << "  - velocity:" << std::endl;
    std::cout << "    - v_0: " << v_v_0.transpose() << std::endl;
    std::cout << "    - w_0: " << v_w_0.transpose() << std::endl;
    std::cout << "    - cov_v_0: " << v_cov_v_0.transpose() << std::endl;
    std::cout << "    - cov_w_0: " << v_cov_w_0.transpose() << std::endl << std::endl;

    std::cout << "Kinematic model:" << std::endl;

    std::cout << "  - position:" << std::endl;
    std::cout << "    - psd_lin_acc: " << psd_lin_acc.transpose() << std::endl;
    std::cout << "    - sigma_ang_vel: " << sigma_ang_vel.transpose() << std::endl << std::endl;

    std::cout << "  - velocity:" << std::endl;
    std::cout << "    - q_v: " << kin_q_v.transpose() << std::endl;
    std::cout << "    - q_w: " << kin_q_w.transpose() << std::endl << std::endl;

    std::cout << "Measurement model:" << std::endl;

    std::cout << "  - position: " << std::endl;
    std::cout << "    - cov_v: " << p_meas_cov_v.transpose() << std::endl;
    std::cout << "    - cov_w: " << p_meas_cov_w.transpose() << std::endl;
    std::cout << "    - cov_x: " << p_meas_cov_x.transpose() << std::endl;
    std::cout << "    - cov_q: " << p_meas_cov_q.transpose() << std::endl;
    std::cout << "    - use_pose_measurement: " << use_pose_measurement << std::endl;
    std::cout << "    - use_pose_resync: " << use_pose_resync << std::endl;
    std::cout << "    - use_velocity_measurement: " << use_velocity_measurement << std::endl << std::endl;

    std::cout << "  - velocity: " << std::endl;
    std::cout << "    - cov_flow: " << v_meas_cov_flow.transpose() << std::endl;
    std::cout << "    - depth_maximum: " << depth_maximum << std::endl;
    std::cout << "    - subsampling_radius: " << subsampling_radius << std::endl;
    std::cout << "    - flow_weighting: " << flow_weighting << std::endl << std::endl;

    std::cout << "Model:" << std::endl;

    std::cout << "- model_name: " << model_name << std::endl;
    std::cout << "- model_use_internal_db: " << model_use_internal_db << std::endl;
    std::cout << "- model_internal_db_name: " << model_internal_db_name << std::endl;
    std::cout << "- model_external_path: " << model_external_path << std::endl << std::endl;

    std::cout << "Optical flow:" << std::endl;

    std::cout << "- source: " << optical_flow_source << std::endl << std::endl;

    std::cout << "Outlier rejection:" << std::endl;

    std::cout << "- enable: " << outlier_rejection_enable << std::endl;
    std::cout << "- gain: " << outlier_rejection_gain << std::endl << std::endl;

    std::cout << "Pose:" << std::endl;

    std::cout << "- source: " << pose_source << std::endl << std::endl;

    std::cout << "Segmentation:" << std::endl;

    std::cout << "- source: " << segmentation_source << std::endl;
    std::cout << "- flow_aided: " << flow_aided_segmentation << std::endl << std::endl;

    std::cout << "Unscented transform:" << std::endl;
    std::cout << "- alpha: " << ut_alpha << std::endl;
    std::cout << "- beta: "  << ut_beta << std::endl;
    std::cout << "- kappa: " << ut_kappa << std::endl << std::endl;

    /* Compose initial condition vectors. */

    VectorXd p_initial_condition(13);
    p_initial_condition.head<3>() = p_v_0;
    p_initial_condition.segment<3>(3) = p_w_0;
    p_initial_condition.segment<3>(6) = Vector3d::Zero();
    Quaterniond q_0 (AngleAxisd(0.0, Vector3d::UnitX()));
    p_initial_condition.tail<4>()(0) = q_0.w();
    p_initial_condition.tail<4>()(1) = q_0.x();
    p_initial_condition.tail<4>()(2) = q_0.y();
    p_initial_condition.tail<4>()(3) = q_0.z();

    VectorXd p_initial_covariance(12);
    p_initial_covariance.head<3>() = p_cov_v_0;
    p_initial_covariance.segment<3>(3) = p_cov_w_0;
    p_initial_covariance.segment<3>(6) = p_cov_x_0;
    p_initial_covariance.tail<3>() = p_cov_q_0;

    VectorXd v_initial_condition(6);
    v_initial_condition.head<3>() = v_v_0;
    v_initial_condition.tail<3>() = v_w_0;

    VectorXd v_initial_covariance(6);
    v_initial_covariance.head<3>() = v_cov_v_0;
    v_initial_covariance.tail<3>() = v_cov_w_0;

    VectorXd p_model_covariance(6);
    p_model_covariance.head<3>() = sigma_ang_vel;
    p_model_covariance.tail<3>() = psd_lin_acc;

    VectorXd v_model_covariance(6);
    v_model_covariance.head<3>() = kin_q_v;
    v_model_covariance.tail<3>() = kin_q_w;

    VectorXd p_measurement_covariance(12);
    p_measurement_covariance.head<3>() = p_meas_cov_v;
    p_measurement_covariance.segment<3>(3) = p_meas_cov_w;
    p_measurement_covariance.segment<3>(6) = p_meas_cov_x;
    p_measurement_covariance.tail<3>() = p_meas_cov_q;

    VectorXd v_measurement_covariance = v_meas_cov_flow;

    /* Object model. */
    ModelParameters model_parameters;
    model_parameters.name(model_name);
    model_parameters.use_internal_db(model_use_internal_db);
    model_parameters.internal_db_name(model_internal_db_name);
    model_parameters.mesh_external_path(model_external_path);

    /* Camera. */
    std::shared_ptr<Camera> camera;
    if (camera_source == "RealSense")
    {
        camera = std::make_shared<RealsenseCameraYarp>(log_name_);
    }
    else if (camera_source == "YARP")
    {
        camera = std::make_shared<YarpCamera>(camera_width, camera_height, camera_fx, camera_cx, camera_fy, camera_cy, log_name_);
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown camera source " + camera_source + "."));

    /* Segmentation. */
    std::shared_ptr<Segmentation> segmentation;
    if (segmentation_source == "YARP")
    {
        segmentation = std::make_shared<SegmentationYarpPort>("/" + log_name_ + "/", true);
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown camera source " + segmentation_source + "."));

    /* Pose. */
    std::shared_ptr<RobotsIO::Utils::Transform> pose;
    if (pose_source == "YARP")
    {
        pose = std::make_shared<TransformYarpPort>("/" + log_name_ + "/pose:i");
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown pose source " + pose_source + "."));

    /* Flow. */
    std::shared_ptr<ImageOpticalFlowSource> flow;
    if (optical_flow_source == "NVOF")
    {
        flow = std::make_shared<OTL::ImageOpticalFlowNVOF>(camera, OTL::ImageOpticalFlowNVOF::NVOFPerformance::Slow, false);
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Error: unknown optical flow source " + pose_source + "."));
}


void Tracker::foo()
{

}


VectorXd Tracker::load_vector_double(const Bottle& resource, const std::string& key, const std::size_t size)
{
    if (resource.find(key).isNull())
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Cannot find key " + key + "."));

    Bottle* b = resource.find(key).asList();
    if (b == nullptr)
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Cannot get vector having key " + key + " as a list."));


    if (b->size() != size)
        throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double. Vector having key " + key + " has size "  + std::to_string(b->size()) + " (expected is " + std::to_string(size) + ")."));

    VectorXd vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double." + std::to_string(i) + "-th element of of vector having key " + key + " is null."));

        if (!item_v.isDouble())
            throw(std::runtime_error("robmo-misc-object-tracker-of::load_vector_double." + std::to_string(i) + "-th element of of vector having key " + key + " is not a double."));

        vector(i) = item_v.asDouble();
    }

    return vector;
}