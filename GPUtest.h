
// Created by guangfu on 2020/10/7.
//

#ifndef TEST_GPUTEST_H
#define TEST_GPUTEST_H



//camera depth usable zone in mm.
#define SMALLEST_DISTANCE_SHOOTING 0.4
#define LARGEST_DISTANCE_SHOOTING 8

//camera intrinsic params.
#define FX 525.0f
#define FY 525.0f
#define CX 319.5f
#define CY 239.5f

#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <boost/compute/system.hpp>
#include <boost/compute/interop/opencv/core.hpp>
#include <boost/compute/interop/opencv/highgui.hpp>
#include <boost/compute/utility/source.hpp>
#include <boost/compute/source.hpp>
#include <boost/compute/container.hpp>
#include <boost/compute/algorithm.hpp>
#include <tbb/tbb.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <CL/cl_platform.h>

namespace compute = boost::compute;

using namespace Eigen;
using namespace cv;
using namespace tbb;
using namespace std;

#define BOOST_COMPUTE_DEBUG_KERNEL_COMPILATION
#define BOOST_COMPUTE_HAVE_THREAD_LOCAL
#define BOOST_COMPUTE_THREAD_SAFE
#define BOOST_COMPUTE_USE_OFFLINE_CACHE

namespace test {

    static void for_loop_test_GPU() {
        // get the default compute device
        compute::device gpu = compute::system::default_device();

        // create a compute context and command queue
        compute::context ctx(gpu);
        compute::command_queue queue(ctx, gpu);

        // generate random numbers on the host
        std::vector<float> host_vector(1000000);
        std::generate(host_vector.begin(), host_vector.end(), rand);

        std::vector<float> cmp_vector(1000000);
        std::generate(cmp_vector.begin(), cmp_vector.end(), rand);

        std::chrono::time_point<std::chrono::steady_clock> s1 = std::chrono::steady_clock::now();

        std::sort(cmp_vector.begin(), cmp_vector.end());

        std::chrono::time_point<std::chrono::steady_clock> e1 = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration1 = e1 - s1;
        float t1 = duration1.count() * 1000.0f;
        std::cout << "naive sorting using stl takes: " << t1 << "ms" << std::endl;

        // create vector on the device
        compute::vector<float> device_vector(1000000, ctx);

        // copy data to the device
        compute::copy(
                host_vector.begin(), host_vector.end(), device_vector.begin(), queue
        );
        std::chrono::time_point<std::chrono::steady_clock> s2 = std::chrono::steady_clock::now();
        // sort data on the device
        compute::sort(
                device_vector.begin(), device_vector.end(), queue
        );

        std::chrono::time_point<std::chrono::steady_clock> e2 = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration2 = e2 - s2;
        float t2 = duration2.count() * 1000.0f;
        std::cout << "sorting using gpu takes: " << t2 << "ms" << std::endl;

        // copy data back to the host
        compute::copy(
                device_vector.begin(), device_vector.end(), host_vector.begin(), queue
        );
        std::chrono::time_point<std::chrono::steady_clock> e3 = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration3 = e3 - e2;
        float t3 = duration3.count() * 1000.0f;
        std::cout << "copying from gpu takes: " << t3 << "ms" << std::endl;
    }

    static void img_for_loop_test_GPU() {
        std::vector<std::vector<Eigen::Vector3f>> current_norms = std::vector<std::vector<Eigen::Vector3f>>(480,
                                                                                                            std::vector<Eigen::Vector3f>(
                                                                                                                    640));
        std::vector<std::vector<Eigen::Vector3f>> current_vertices = std::vector<std::vector<Eigen::Vector3f>>(480,
                                                                                                               std::vector<Eigen::Vector3f>(
                                                                                                                       640));
        std::vector<std::vector<bool>> validPixelCache = std::vector<std::vector<bool>>(480,
                                                                                        std::vector<bool>(640, false));

        cv::Mat depth = cv::Mat(cv::Size(640, 480), CV_16UC1);
        depth = cv::imread("/home/guangfu/Documents/d2.png", -1);

        std::chrono::time_point<std::chrono::steady_clock> s1 = std::chrono::steady_clock::now();
        parallel_for(tbb::blocked_range<size_t>(0, 479), [&](const tbb::blocked_range<size_t> &range) {
            for (int r = range.begin(); r != range.end(); r++) {
                for (int c = 0; c < 639; c++) {
                    float d = depth.at<ushort>(r, c);
                    if (d >= 5 * SMALLEST_DISTANCE_SHOOTING && d <= 5 * LARGEST_DISTANCE_SHOOTING) {
                        //TODO: here generate first frame norms and vertices.
                        validPixelCache[r][c] = true;
                        current_vertices[r][c] = Eigen::Vector3f(d * ((float) (c - CX) / FX),
                                                                 d * ((float) (r - CY) / FY), d);
                    }
                }
            }
        }, tbb::auto_partitioner());

        parallel_for(tbb::blocked_range<size_t>(0, 479), [&](const tbb::blocked_range<size_t> &range) {
            for (int r = range.begin(); r != range.end(); r++) {
                for (int c = 0; c != 639; c++) {
                    if (validPixelCache[r][c]) {
                        current_norms[r][c] = (current_vertices[r][c + 1] - current_vertices[r][c]).cross(
                                current_vertices[r + 1][c] - current_vertices[r][c]);
                        current_norms[r][c] = current_norms[r][c] / current_norms[r][c].norm();
                    }
                }
            }
        }, tbb::auto_partitioner());

        std::chrono::time_point<std::chrono::steady_clock> e1 = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration1 = e1 - s1;
        float t1 = duration1.count() * 1000.0f;
        std::cout << "tbb computing takes: " << t1 << "ms" << std::endl;

        //TODO: HERE COMPUTING WITH GPU SUPPORT.
        // get the default compute device
        compute::device gpu = compute::system::default_device();
        // create a compute context and command queue
        compute::context ctx(gpu);
        compute::command_queue queue(ctx, gpu);

//        std::cout<<ctx.get_device().platform().vendor()<<std::endl;
        compute::vector<cl_float3> verts = compute::vector<cl_float3>(480 * 640,ctx);
        //compute::vector<cl_float3> norms = compute::vector<cl_float3>(480 * 640);

        std::vector<cl_float3> o_verts = std::vector<cl_float3>(480 * 640);
        //std::vector<Eigen::Vector3f> o_norms = std::vector<Eigen::Vector3f>(480 * 640);

        depth.convertTo(depth,CV_8U);

        std::chrono::time_point<std::chrono::steady_clock> s2 = std::chrono::steady_clock::now();

        // transfer image to gpu
        compute::image2d input_image =
                compute::opencv_create_image2d_with_mat(
                        depth, compute::image2d::read_write, queue
                );

        const unsigned int h = 480;
        const unsigned int w = 640;

        const char source[] = BOOST_COMPUTE_STRINGIZE_SOURCE(
                __kernel void local_vert_cvt(__read_only image2d_t input,
                                             __global float3* output)
        {
            int u = get_global_id(0);
            int v = get_global_id(1);
            const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;

            const float CX_ = 319.5f;
            const float CY_ = 239.5f;
            const float FX_ = 525.0f;
            const float FY_ = 525.0f;

            float3 vert = {0, 0, 0};

            vert.z = read_imagef(input, sampler, (int2)(u, v)).x;
            vert.y = vert.z * (v - CY_) / FY_;
            vert.x = vert.z * (u - CX_) / FX_;

            if (vert.z >= 5 * 500 / 256 && vert.z <= 5 * 8000 / 256) {
                output[(640 * v + u)] = vert;
            }

        });

        compute::program img_program =
                compute::program::create_with_source(source, ctx);
        img_program.build();

        // create img kernel and set arguments
        compute::kernel img_kernel(img_program, "local_vert_cvt");
        img_kernel.set_arg(0, input_image);
        img_kernel.set_arg(1, verts.get_buffer());

        // run img kernel
        size_t origin[2] = {0, 0};
        size_t region[2] = {w, h};

        queue.enqueue_nd_range_kernel(img_kernel,2, origin, region, 0);

        queue.finish();
        std::chrono::time_point<std::chrono::steady_clock> e2 = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration2 = e2 - s2;
        float t2 = duration2.count() * 1000.0f;
        std::cout << "boost computing using AMD GPU takes: " << t2 << "ms" << std::endl;

        compute::copy(verts.begin(), verts.end(), o_verts.begin(), queue);

//        std::chrono::time_point<std::chrono::steady_clock> e2 = std::chrono::steady_clock::now();
//        std::chrono::duration<float> duration2 = e2 - s2;
//        float t2 = duration2.count() * 1000.0f;
//        std::cout << "boost computing using AMD GPU takes: " << t2 << "ms" << std::endl;
}


static void listDevice() {
    auto devices = boost::compute::system::devices();
    for (auto &d:devices) {
        std::cout << d.name() << " " << d.clock_frequency() << " " << d.compute_units() << std::endl;
    }
    compute::device device = compute::system::default_device();
    std::cout << device.platform().vendor() << std::endl;
}

}


#endif //TEST_GPUTEST_H
