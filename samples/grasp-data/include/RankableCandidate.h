#ifndef RANKABLE_CANDIDATE_H
#define RANKABLE_CANDIDATE_H

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

typedef std::tuple<std::string, double, yarp::sig::Matrix, yarp::sig::Vector> rankable_candidate;

#endif
