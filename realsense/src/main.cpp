/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <cstdlib>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>

#include <RobotsIO/Utils/YarpImageOfProbe.hpp>

using namespace yarp::sig;
using namespace RobotsIO::Utils;


int main(int argc, char ** argv)
{
    /* Create pipeline. */
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 60);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 60);
    pipe.start(cfg);

    /* Create instance of alignment procedure. */
    rs2::align align_to_color(RS2_STREAM_COLOR);

    /* Storage for rgb, depth and time stamps. */
    std::vector<cv::Mat> rgb_frames;
    std::vector<cv::Mat> depth_frames;
    std::vector<double> stamps;

    /* Storage for output. */
    cv::Mat output_rgb;
    cv::Mat output_depth;

    /* YARP output ports. */
    YarpImageOfProbe<PixelRgb> yarp_output_rgb("/roft-realsense/rgb:o");
    YarpImageOfProbe<PixelFloat> yarp_output_depth("/roft-realsense/depth:o");

    /* While loop for data collection. */
    rs2::colorizer c;
    while (true)
    {
        /* Local timer. */
        auto time0 = std::chrono::steady_clock::now();

        /* Wait for new frames. */
        rs2::frameset frameset = pipe.wait_for_frames();

        /* Align frames. */
        frameset = align_to_color.process(frameset);

        /* Extract separate frames and colorize. */
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        /* Not sure it is required, however it stabilizes frame rate! */
        auto colorized_depth = c.colorize(depth);

        /* Wrap data around OpenCV matrices. */
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        cv::Mat cv_rgb(cv::Size(w, h), CV_8UC3, (void*) color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat cv_depth(cv::Size(w, h), CV_16U, (void*) depth.get_data(), cv::Mat::AUTO_STEP);
        cv_depth.convertTo(cv_depth, CV_32FC1, 0.001);

        output_rgb = cv_rgb.clone();
        cv::cvtColor(output_rgb, output_rgb, CV_RGB2BGR);
        output_depth = cv_depth.clone();

        yarp_output_rgb.set_data(output_rgb);
        yarp_output_depth.set_data(output_depth);
    }

    return EXIT_SUCCESS;
}
