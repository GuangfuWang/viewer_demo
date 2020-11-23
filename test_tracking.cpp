//
// Created by root on 2020/10/10.
//

#include "test_tracking.h"
#include <functional>

using tbb::parallel_for;
using tbb::blocked_range;

class PointPlaneTracking;

typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,6,6> Vector66f;

Tracking* Tracking::createTrackingTool( Tracking::TrackingMethod t_method) {
    if (t_method == TrackingMethod::POINT_PLANE) {
        Tracking* tracker = new PointPlaneTracking();
        tracker->init();
        std::cout << "Point plane tracking method used!" << std::endl;
        return tracker;
    }
    std::cerr << "currently only point plane method is support!" << std::endl;
    throw std::exception();
}


bool
PointPlaneTracking::init() {
    if (inited) {
        std::cerr
                << "tracking object have been inited, please use setImages to set image pairs and use iteration method to get pose"
                << std::endl;
        return false;
    }

    this->rgb = cv::Mat(cv::Size(640, 480), CV_8UC3);
    this->depth = cv::Mat(cv::Size(640, 480), CV_16UC1);

    this->iteration_max = 5;
    this->first_frame = true;

    this->delta_pose_vec = Eigen::Matrix<float, 6, 1>::Zero(6, 1);
    this->pose_previous_frame = Eigen::Matrix<float, 4, 4>::Identity(4, 4);
    this->pose_previous_frame_inverse = Eigen::Matrix<float, 4, 4>::Identity(4, 4);
    this->global_pose_z_minus_1_iteration = Eigen::Matrix<float, 4, 4>::Identity(4, 4);
    this->local_pose = Eigen::Matrix<float, 4, 4>::Identity(4, 4);
    this->incremental_delta_pose = Eigen::Matrix<float, 4, 4>::Identity(4, 4);

    this->pose_posterior = Eigen::Matrix<float, 4, 4>::Identity(4, 4);

    //around 3.52MB for each vec. 3*4*480*640 bytes~=3.52MB.
    this->current_norms = vector<vector<Vector3f>,Eigen::aligned_allocator<Vector3f>>(480,
                                                                                      vector<Vector3f>(640));
    this->current_vertices = vector<vector<Vector3f>,Eigen::aligned_allocator<Vector3f>>(480,
                                                                                         vector<Vector3f>(640));

    this->validPixelCache = std::vector<std::vector<bool>>(480, std::vector<bool>(640, false));

    this->intrinsic = Eigen::Matrix<float, 3, 3>::Zero(3, 3);
    this->intrinsic(0, 0) = FX;
    this->intrinsic(1, 1) = FY;
    this->intrinsic(0, 2) = CX;
    this->intrinsic(1, 2) = CY;
    this->intrinsic(2, 2) = 1.0f;
    this->inited = true;
    std::cout<<"inited!"<<std::endl;
    return inited;
}



Eigen::Matrix<float,4,4>
PointPlaneTracking::iterationForPose() {
    std::vector<std::vector<Eigen::Vector3f>,Eigen::aligned_allocator<Vector3f>> c_norms =
            vector<vector<Vector3f>,Eigen::aligned_allocator<Vector3f>>(480,vector<Vector3f>(640));
    std::vector<std::vector<Eigen::Vector3f>,Eigen::aligned_allocator<Vector3f>> c_vertices =
            vector<vector<Vector3f>,Eigen::aligned_allocator<Vector3f>>(480,vector<Vector3f>(640));

    std::vector<std::vector<bool>> cache=
            std::vector<std::vector<bool>>(480, std::vector<bool>(640, false));

    if (first_frame) {
        first_frame = false;
        parallel_for(tbb::blocked_range<size_t>(0, 480), [&](const tbb::blocked_range<size_t> &range) {
            for (int r = range.begin(); r != range.end(); r++) {
                for (int c = 0; c < 640; c++) {
                    float d = this->depth.at<ushort>(r, c)/1000.0f;
                    if (d >= 5 * SMALLEST_DISTANCE_SHOOTING && d <= 5 * LARGEST_DISTANCE_SHOOTING) {
                        //TODO: here generate first frame norms and vertices.
                        cache[r][c] = true;
                        c_vertices[r][c] = Eigen::Vector3f(d * ((float) (c - CX) / FX),
                                                                 d * ((float) (r - CY) / FY), d);
                    }
                }
            }
        }, tbb::auto_partitioner());

        parallel_for(tbb::blocked_range<size_t>(0, 479), [&](const tbb::blocked_range<size_t> &range) {
            for (int r = range.begin(); r != range.end(); r++) {
                for (int c = 0; c != 639; c++) {
                    if (cache[r][c]&&cache[r][c+1]&&cache[r+1][c]) {
                        c_norms[r][c] = (c_vertices[r][c + 1] - c_vertices[r][c]).cross(
                                c_vertices[r + 1][c] - c_vertices[r][c]);
                        c_norms[r][c] = c_norms[r][c] / c_norms[r][c].norm();
                    }
                }
            }
        }, tbb::auto_partitioner());

        c_norms.swap(this->current_norms);
        c_vertices.swap(this->current_vertices);
        cache.swap(this->validPixelCache);

        //std::cerr << "first frame pose will be set to Identity..." << std::endl;
        return Eigen::Matrix<float,4,4>::Identity();
    } else {
        //calculate norms and vertices

        parallel_for(tbb::blocked_range<size_t>(0, 480), [&](const tbb::blocked_range<size_t> &range)  {
            for (int r = range.begin(); r != range.end(); r++) {
                for (int c = 0; c < 640; c++) {
                    float d = depth.at<ushort>(r, c)/1000.0f;
                    if (d >= 5 * SMALLEST_DISTANCE_SHOOTING && d <= 5 * LARGEST_DISTANCE_SHOOTING) {
                        //TODO: here generate first frame norms and vertices.
                        cache[r][c] = true;
                        c_vertices[r][c] = Eigen::Vector3f(d * ((float) (c - CX) / FX),
                                                                 d * ((float) (r - CY) / FY), d);
                    }
                }
            }
        }, tbb::auto_partitioner());

        parallel_for(tbb::blocked_range<size_t>(0, 479), [&](const tbb::blocked_range<size_t> &range) {
            for (int r = range.begin(); r != range.end(); r++) {
                for (int c = 0; c != 639; c++) {
                    if (cache[r][c]&&cache[r][c+1]&&cache[r+1][c]) {
                        c_norms[r][c] = (c_vertices[r][c + 1] - c_vertices[r][c]).cross(
                                c_vertices[r + 1][c] - c_vertices[r][c]);
                        c_norms[r][c] = c_norms[r][c] / c_norms[r][c].norm();
                    }
                }
            }
        }, tbb::auto_partitioner());

        //start iteration
        //first load to A and b, then solve linear equation iteratively by cholesky decomposition.
        Eigen::Matrix<float, 6, 1> b_r = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 6> A_M = Eigen::Matrix<float, 6, 6>::Zero();

        vector<Vector6f,Eigen::aligned_allocator<Vector6f>> vec_b_r=
                vector<Vector6f,Eigen::aligned_allocator<Vector6f>>(480*640,Vector6f::Zero(6,1));
        vector<Vector66f,Eigen::aligned_allocator<Vector66f>> vec_A_M=
                vector<Vector66f,Eigen::aligned_allocator<Vector66f>>(480*640,Vector66f::Zero(6,6));

        tbb::task_group g;
        Vector6f s;
        Vector66f s_M;
        while (currentIteration < iteration_max) {
            tbb::parallel_for(blocked_range<size_t>(0, 479), [&](const tbb::blocked_range<size_t> &range) {
                for (int r = range.begin(); r != range.end(); r++) {
                    for (int c = 0; c < 639; c++) {
                        if (cache[r][c]&&cache[r][c+1]&&cache[r+1][c]) {
                            Eigen::Matrix<float, 1, 6> A_ = Eigen::Matrix<float, 1, 6>::Zero();
                            Eigen::Matrix<float,3,6> skew_M = Eigen::Matrix<float,3,6>::Zero(3,6);
                            skew_M(0, 3) = 1.0f;
                            skew_M(1, 4) = 1.0f;
                            skew_M(2, 5) = 1.0f;
                            //u^
                            Eigen::Vector3f v_pseudoPos = intrinsic*(local_pose*Eigen::Vector4f(c_vertices[r][c].x(),c_vertices[r][c].y(),c_vertices[r][c].z(),1.0f)).block<3,1>(0,0);
                            Eigen::Vector2i pseudoPos;
                            if(std::abs(v_pseudoPos[2])>EPSILON){
                                int u=std::round(v_pseudoPos[0]/v_pseudoPos[2]);
                                int v=std::round(v_pseudoPos[1]/v_pseudoPos[2]);
                                if(u>=0&&u<640&&v>=0&&v<480){
                                    pseudoPos=Eigen::Vector2i(u,v);
                                    if(current_norms[pseudoPos[1]][pseudoPos[0]].norm()>EPSILON&&
                                       validPixelCache[pseudoPos[1]][pseudoPos[0]]){
                                        //N^g k-1.
                                        Eigen::Vector3f norm_=pose_previous_frame.block<3,3>(0,0)*current_norms[pseudoPos[1]][pseudoPos[0]];
                                        norm_/=norm_.norm();
                                        Eigen::Vector4f s_v=global_pose_z_minus_1_iteration *
                                                            Eigen::Vector4f(c_vertices[r][c].x(), c_vertices[r][c].y(), c_vertices[r][c].z(), 1.0f);
                                        skew_M(0, 1) = -s_v.z();
                                        skew_M(1, 0) = +s_v.z();
                                        skew_M(1, 2) = -s_v.x();
                                        skew_M(2, 1) = +s_v.x();
                                        skew_M(0, 2) = +s_v.y();
                                        skew_M(2, 0) = -s_v.y();

                                        A_+=norm_.transpose()*skew_M;

                                        Eigen::Vector4f vert_ = pose_previous_frame*Eigen::Vector4f(
                                                current_vertices[pseudoPos[1]][pseudoPos[0]].x(),current_vertices[pseudoPos[1]][pseudoPos[0]].y(),
                                                current_vertices[pseudoPos[1]][pseudoPos[0]].z(),1.0f) ;
                                        Eigen::Vector4f global_posterior_vert = global_pose_z_minus_1_iteration*Eigen::Vector4f(
                                                c_vertices[r][c].x(),c_vertices[r][c].y(),
                                                c_vertices[r][c].z(),1.0f);

                                        vert_ -= global_posterior_vert;
                                        float temp_b = norm_[0] * vert_[0] + norm_[1] * vert_[1] + norm_[2] * vert_[2];
                                        vec_b_r[640*r+c]=A_.transpose() * temp_b;
                                        vec_A_M[640*r+c]=A_.transpose() * A_;
                                    }
                                }
                            }
                        }
                    }

                }
            },tbb::auto_partitioner());

            s=Vector6f::Zero();
            s_M=Vector66f::Zero();

            g.run([&](){
                b_r=tbb::parallel_reduce(
                        tbb::blocked_range<vector<Vector6f,Eigen::aligned_allocator<Vector6f>>::iterator>(vec_b_r.begin(),vec_b_r.end()),
                        s,
                        [&](const tbb::blocked_range<vector<Vector6f,Eigen::aligned_allocator<Vector6f>>::iterator>& r,Vector6f init )->Vector6f {
                            for(auto each = r.begin();each!=r.end();each++){
                                init+=*each;
                            }
                            return init;
                        },
                        std::plus<Vector6f>()
                );
            });
            g.run([&](){
                A_M = tbb::parallel_reduce(
                        tbb::blocked_range<vector<Vector66f,Eigen::aligned_allocator<Vector66f>>::iterator>(vec_A_M.begin(),vec_A_M.end()),
                        s_M,
                        [](const tbb::blocked_range<vector<Vector66f,Eigen::aligned_allocator<Vector66f>>::iterator>& r,Vector66f init )->Vector66f {
                            for(auto each=r.begin();each!=r.end();each++){
                                init+=(*each);
                            }
                            return init;
                        },
                        std::plus<Vector66f>());
            });
            g.wait();
            delta_pose_vec = A_M.ldlt().solve(b_r);
            incremental_delta_pose(0, 1) =  delta_pose_vec(2, 0);
            incremental_delta_pose(1, 0) = -delta_pose_vec(2, 0);
            incremental_delta_pose(2, 0) =  delta_pose_vec(1, 0);
            incremental_delta_pose(0, 2) = -delta_pose_vec(1, 0);
            incremental_delta_pose(1, 2) =  delta_pose_vec(0, 0);
            incremental_delta_pose(2, 1) = -delta_pose_vec(0, 0);
            incremental_delta_pose(0, 3) =  delta_pose_vec(3, 0);
            incremental_delta_pose(1, 3) =  delta_pose_vec(4, 0);
            incremental_delta_pose(2, 3) =  delta_pose_vec(5, 0);

            pose_posterior=incremental_delta_pose*pose_posterior;
            local_pose=pose_previous_frame_inverse*pose_posterior;
            global_pose_z_minus_1_iteration = pose_posterior;

            currentIteration++;
        }
        c_norms.swap(this->current_norms);
        c_vertices.swap(this->current_vertices);
        cache.swap(this->validPixelCache);
    }
    return pose_posterior;
}

void
PointPlaneTracking::setImagePairs(cv::Mat &rgb, cv::Mat &depth, double &timestamp) {
    this->currentIteration = 0;
    this->timestamp = timestamp;
    this->rgb = rgb;
    this->depth = depth;
    this->pose_previous_frame = this->pose_posterior;
    this->pose_previous_frame_inverse = this->pose_previous_frame.inverse();
    this->global_pose_z_minus_1_iteration = this->pose_previous_frame;
    //TODO: test if using local pose estimation is better or not.
    this->local_pose = Eigen::Matrix<float, 4, 4>::Identity(4, 4);
}
