/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef MODULE_H
#define MODULE_H

#include <Eigen/Dense>
#include <viewer.h>

#include <yarp/os/RFModule.h>


class Module : public yarp::os::RFModule
{
public:
    bool configure(yarp::os::ResourceFinder& rf) override;

    bool updateModule() override;

private:
    // void show_grasp_candidates(const std::string& name, const Eigen::Transform<double, 3, Eigen::Affine>& pose, const std::vector<cardinal_points_grasp::rankable_candidate>& candidates);

    std::shared_ptr<viewer::Viewer> viewer_;

    const std::string log_name_ = "roft-grasp-viewer";
};

#endif /* MODULE_H */
