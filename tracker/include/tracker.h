/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <thrift/TrackerIDL.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>

#include <Eigen/Dense>


class Tracker : public TrackerIDL
{
public:
    Tracker(const yarp::os::ResourceFinder& resource_finder);

    /**
     * IDL interface.
     */

    void foo() override;

private:
    Eigen::VectorXd load_vector_double(const yarp::os::Bottle& resource, const std::string& key, const std::size_t size);

    const std::string log_name_ = "roft-tracker";
};

#endif /* TRACKER_H */
