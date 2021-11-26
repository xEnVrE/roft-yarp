/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <module.h>

// #include <yarp/eigen/Eigen.h>
// #include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
// #include <yarp/math/Math.h>

using namespace Eigen;
using namespace viewer;
// using namespace yarp::eigen;
using namespace yarp::os;
// using namespace yarp::sig;
// using Pose = Eigen::Transform<double, 3, Eigen::Affine>;


bool Module::configure(yarp::os::ResourceFinder& rf)
{
    /* Get parameters. */
    const double period = rf.check("period", Value(30)).asInt();
    const std::string object_meshes_path = rf.check("meshes_path", Value("")).asString();
    const int width = rf.check("width", Value(320)).asInt();
    const int height = rf.check("height", Value(320)).asInt();

    /* Set object properties. */
    std::unordered_map<std::string, Eigen::VectorXd> object_properties;

    object_properties["003_cracker_box"] = Eigen::Vector3d::Zero();
    object_properties.at("003_cracker_box")(0) = 0.0718;
    object_properties.at("003_cracker_box")(1) = 0.1640;
    object_properties.at("003_cracker_box")(2) = 0.2134;

    object_properties["004_sugar_box"] = Eigen::Vector3d::Zero();
    object_properties.at("004_sugar_box")(0) = 0.0451;
    object_properties.at("004_sugar_box")(1) = 0.0927;
    object_properties.at("004_sugar_box")(2) = 0.1763;

    object_properties["006_mustard_bottle"] = Eigen::Vector3d::Zero();
    object_properties.at("006_mustard_bottle")(0) = 0.0582;
    object_properties.at("006_mustard_bottle")(1) = 0.0960;
    object_properties.at("006_mustard_bottle")(2) = 0.1913;

    /* Initialize the viewer and load the objects. */
    viewer_ = std::make_shared<Viewer>(10, 370, width, height);
    viewer_->loadObjects(object_properties, object_meshes_path);

    /* Start the viewer. */
    viewer_->start(period);

    return true;
}


bool Module::updateModule()
{
    return true;
}


// void Module::show_grasp_candidates
// (
//     const std::string& name,
//     const Eigen::Transform<double, 3, Eigen::Affine>& pose,
//     const std::vector<cardinal_points_grasp::rankable_candidate>& candidates
// )
// {
//     yarp::sig::Vector cam_x;
//     yarp::sig::Vector cam_o;
//     gaze_->controller().getLeftEyePose(cam_x, cam_o);

//     Bottle info;
//     gaze_->controller().getInfo(info);
//     const auto w = info.find("camera_width_left").asInt();
//     const auto fov_h = info.find("camera_intrinsics_left").asList()->get(0).asFloat64();
//     const auto view_angle = 2. * std::atan((w / 2.) / fov_h) * (180. / M_PI);

//     viewer_->addCamera
//     (
//         {cam_x[0], cam_x[1], cam_x[2]},
//         {pose.translation()(0), pose.translation()(1), pose.translation()(2)},
//         {0., 0., 1.},
//         view_angle
//     );
//     viewer_->addObject(object_name_, pose);
//     viewer_->focusOnObject();
//     viewer_->showCandidates(candidates);
// }
