//
// Created by root on 2020/10/10.
//

//
// Created by root on 2020/9/29.
//

#ifndef FUSION_TRACKING_H
#define FUSION_TRACKING_H



//camera depth usable zone in mm.
#define SMALLEST_DISTANCE_SHOOTING 0.4
#define LARGEST_DISTANCE_SHOOTING 8

//camera intrinsic params.
#define FX 525.0f
#define FY 525.0f
#define CX 319.5f
#define CY 239.5f

//calculation threshold error.
#define EPSILON 1.0E-40

#include <tbb/tbb.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <immintrin.h>
#include <vector>

#ifndef USE_AVX2
#define USE_AVX2
#endif

//aligned bytes of 32 due to AVX2. if using SSE 16 bytes should be aligned.
#ifdef USE_AVX2
#define ALIGNED __attribute__ ((aligned(32)))
#endif

using Eigen::Matrix;
using Eigen::Vector3f;
using Eigen::Matrix4f;
using Eigen::Vector2i;
using std::vector;
using std::cout;
using std::endl;

class Tracking {
protected:
    double timestamp;
    unsigned char currentIteration;
    mutable unsigned char iteration_max = 5;
    bool first_frame = true;
    bool inited = false;

    cv::Mat depth;
    cv::Mat rgb;

    Eigen::Matrix<float, 6, 1> delta_pose_vec;
    Eigen::Matrix<float, 4, 4> global_pose_z_minus_1_iteration;
    Eigen::Matrix<float, 4, 4> incremental_delta_pose;
    Eigen::Matrix<float, 4, 4> pose_posterior;
    Eigen::Matrix<float, 4, 4> pose_previous_frame;
    Eigen::Matrix<float, 4, 4> pose_previous_frame_inverse;
    Eigen::Matrix<float, 4, 4> local_pose;


    std::vector<std::vector<Eigen::Vector3f>,Eigen::aligned_allocator<Eigen::Vector3f>> current_norms;
    std::vector<std::vector<Eigen::Vector3f>,Eigen::aligned_allocator<Eigen::Vector3f>> current_vertices;

    std::vector<std::vector<bool>> validPixelCache;

    Eigen::Matrix<float, 3, 3> intrinsic;

public:
    enum class TrackingMethod {
        POINT_PLANE = 0,
        POINT_POINT = 1,
        DEPTH_ITERATION = 2,
        GRAY_SCALE_ITERATION = 3
    };

    enum class USE_SIMD {
        WITHOUT_SIMD = 7,
        SSE = 0,
        SSE2 = 1,
        SSE3 = 2,
        SSE4_1 = 3,
        SSE4_2 = 4,
        AVX = 5,
        AVX2 = 6
    };

    inline static
    Eigen::Vector4f cvt2HomoVec4(float &x, float &y, float &z) {
        Eigen::Vector4f ret = Eigen::Vector4f(x, y, z, 1);
        return ret;
    }

    inline static
    Eigen::Vector2i correspondingPixel(Eigen::Matrix<float, 3, 3> &intrinsic,
                                       Eigen::Matrix<float, 4, 4> &local_pose,
                                       float &x, float &y, float &z) {

        ALIGNED float pose_0[8] = {
                local_pose(0, 0), local_pose(0, 1), local_pose(0, 2), local_pose(0, 3),
                local_pose(1, 0), local_pose(1, 1), local_pose(1, 2), local_pose(1, 3)
        };
        ALIGNED float pose_1[8] = {
                local_pose(2, 0), local_pose(2, 1), local_pose(2, 2), local_pose(2, 3),
                0.0f, 0.0f, 0.0f, 1.0f
        };

        ALIGNED float p[8] = {
                x, y, z, 1.0f,
                x, y, z, 1.0f
        };

        ALIGNED float res_0[8] = {0};
        ALIGNED float res_1[8] = {0};
        __m256 arr_m0 = _mm256_load_ps(pose_0);
        __m256 arr_m1 = _mm256_load_ps(pose_1);
        __m256 arr_m2 = _mm256_load_ps(p);

        __m256 res_m0 = _mm256_mul_ps(arr_m0, arr_m2);
        __m256 res_m1 = _mm256_mul_ps(arr_m1, arr_m2);

        _mm256_store_ps(res_0, res_m0);
        _mm256_store_ps(res_1, res_m1);
        ALIGNED float intermdiate[8] = {0};
        intermdiate[0] = intermdiate[4] = res_0[0] + res_0[1] + res_0[2] + res_0[3];
        intermdiate[1] = intermdiate[5] = res_0[4] + res_0[5] + res_0[6] + res_0[7];
        intermdiate[2] = intermdiate[6] = res_1[0] + res_1[1] + res_1[2] + res_1[3];

        ALIGNED float arr_pose_0[8] = {
                intrinsic(0, 0), intrinsic(0, 1), intrinsic(0, 2), 0.0f,
                intrinsic(1, 0), intrinsic(1, 1), intrinsic(1, 2), 0.0f
        };

        ALIGNED float arr_pose_1[8] = {
                intrinsic(2, 0), intrinsic(2, 1), intrinsic(2, 2), 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f
        };

        arr_m0 = _mm256_load_ps(intermdiate);
        arr_m1 = _mm256_load_ps(arr_pose_0);
        arr_m2 = _mm256_load_ps(arr_pose_1);
        res_m0 = _mm256_mul_ps(arr_m0, arr_m1);
        res_m1 = _mm256_mul_ps(arr_m0, arr_m2);
        _mm256_store_ps(res_0, res_m0);
        _mm256_store_ps(res_1, res_m1);

        float x_ = res_0[0] + res_0[1] + res_0[2];
        float y_ = res_0[4] + res_0[5] + res_0[6];
        float z_ = res_1[0] + res_1[1] + res_1[2];

        if (std::abs(z_) < EPSILON) {
//            std::cerr << "there is something wrong during converting a local point to its previous corresponding point!"
//                      << std::endl;
            return Eigen::Vector2i(-1, -1);
        }

        int u = (int) (x_ / z_);
        int v = (int) (y_ / z_);
        if (u < 0 || u >= 640 || v < 0 || v >= 480) {
//            std::cerr << "this point is NOT in frustum..." << std::endl;
            return Eigen::Vector2i(-1, -1);
        }
        //std::cout << "converting a local point to its previous frame point: Done..." << std::endl;

        return Eigen::Vector2i(u, v);
    }

    inline bool
    changeIterationMax(unsigned char new_iteration_max) const {
        if (new_iteration_max < iteration_max) {
            std::cerr << "new iteration number is smaller than 5, which is NOT acceptable!!!" << std::endl;
            return false;
        }
        iteration_max = new_iteration_max;
        return true;
    }

    inline static
    Eigen::Vector2i PI_2i(float &x_, float &y_, float &z_) {
        if (std::abs(z_) < EPSILON) {
            std::cerr << "depth value is too small, pixel (-1,-1) will be returned..." << std::endl;
            return Eigen::Vector2i(-1, -1);
        }
        return Eigen::Vector2i(x_ / z_, y_ / z_);
    }

    inline  static bool
    validPixel(int &v, int &u,cv::Mat& depth) {
        if (depth.at<ushort>(v, u) < 5*SMALLEST_DISTANCE_SHOOTING ||
            depth.at<ushort>(v, u) > 5*LARGEST_DISTANCE_SHOOTING) {
            return false;
        }
        return true;
    }

    inline static Eigen::Vector3f
    pixel_2_local_point(int &v, int &u, const cv::Mat &depth) {
        Eigen::Vector3f ret = Eigen::Vector3f::Zero(3, 1);
        ret[0] = (float) depth.at<ushort>(v, u) * ((float) (u - CX) / FX);
        ret[1] = (float) depth.at<ushort>(v, u) * ((float) (v - CY) / FY);
        ret[2] = (float) depth.at<ushort>(v, u);
        return ret;
    }

    inline static Eigen::Vector3f
    local_norm_2_global_norm(Eigen::Matrix<float, 4, 4> &_p, Eigen::Vector3f &local_norm) {
        Eigen::Vector3f ret = Eigen::Vector3f::Zero(3,1);

        ALIGNED float arr_p_0[8]={
                _p(0, 0), _p(0, 1), _p(0, 2), 0.0f,
                _p(1, 0), _p(1, 1), _p(1, 2), 0.0f
        };

        ALIGNED float arr_p_1[8]={
                _p(2, 0), _p(2, 1), _p(2, 2), 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f
        };

        ALIGNED float arr_n[8]={
                local_norm[0], local_norm[1], local_norm[2], 0.0f,
                local_norm[0], local_norm[1], local_norm[2], 0.0f
        };

        __m256 a_0=_mm256_load_ps(arr_p_0);
        __m256 a_1=_mm256_load_ps(arr_p_1);
        __m256 n_0=_mm256_load_ps(arr_n);

        __m256 r_0=_mm256_mul_ps(a_0,n_0);
        __m256 r_1=_mm256_mul_ps(a_1,n_0);

        ALIGNED float res_0[8]={0};
        ALIGNED float res_1[8]={0};

        _mm256_store_ps(res_0,r_0);
        _mm256_store_ps(res_1,r_1);

        ret[0]=res_0[0]+res_0[1]+res_0[2];
        ret[1]=res_0[4]+res_0[5]+res_0[6];
        ret[2]=res_1[0]+res_1[1]+res_1[2];

        return ret;
    }

    inline static Eigen::Vector3f
    local_vert_2_global_vert(Eigen::Matrix<float, 4, 4> &_p, Eigen::Vector3f &local_vert) {
        Eigen::Vector3f ret = Eigen::Vector3f::Zero(3,1);
        ALIGNED float arr_p_0[8]={
                _p(0, 0), _p(0, 1), _p(0, 2), _p(0, 3),
                _p(1, 0), _p(1, 1), _p(1, 2), _p(1, 3)
        };
        ALIGNED float arr_p_1[8]={
                _p(2, 0), _p(2, 1), _p(2, 2), _p(2, 3),
                0.0f, 0.0f, 0.0f, 0.0f
        };
        ALIGNED float arr_v[8]={
                local_vert[0], local_vert[1], local_vert[2], 1.0f,
                local_vert[0], local_vert[1], local_vert[2], 1.0f
        };

        __m256 arr_m0=_mm256_load_ps(arr_p_0);
        __m256 arr_m1=_mm256_load_ps(arr_p_1);
        __m256 arr_v0=_mm256_load_ps(arr_v);

        __m256 r_0=_mm256_mul_ps(arr_m0,arr_v0);
        __m256 r_1=_mm256_mul_ps(arr_m1,arr_v0);

        ALIGNED float res_0[8]={0};
        ALIGNED float res_1[8]={0};

        _mm256_store_ps(res_0,r_0);
        _mm256_store_ps(res_1,r_1);

        ret[0]=res_0[0]+res_0[1]+res_0[2]+res_0[3];
        ret[1]=res_0[4]+res_0[5]+res_0[6]+res_0[7];
        ret[2]=res_1[0]+res_1[1]+res_1[2]+res_1[3];

        return ret;
    }

    inline static Eigen::Matrix<float, 1, 6>
    skew_norm(Eigen::Matrix<float, 3, 6> &skew_matrix, Eigen::Vector3f &norm_) {
        Eigen::Matrix<float, 1, 6> ret;
        ALIGNED float arr1_0[8] = {
                skew_matrix(0, 0), skew_matrix(1, 0), skew_matrix(2, 0), 0.0f,
                skew_matrix(0, 1), skew_matrix(1, 1), skew_matrix(2, 1), 0.0f
        };
        ALIGNED float arr1_1[8] = {
                skew_matrix(0, 2), skew_matrix(1, 2), skew_matrix(2, 2), 0.0f,
                skew_matrix(0, 3), skew_matrix(1, 3), skew_matrix(2, 3), 0.0f
        };
        ALIGNED float arr1_2[8] = {
                skew_matrix(0, 4), skew_matrix(1, 4), skew_matrix(2, 4), 0.0f,
                skew_matrix(0, 5), skew_matrix(1, 5), skew_matrix(2, 5), 0.0f
        };
        ALIGNED float arr2[8] = {
                norm_[0], norm_[1], norm_[2], 0.0f,
                norm_[0], norm_[1], norm_[2], 0.0f
        };

        __m256 r1_0 = _mm256_load_ps(arr1_0);
        __m256 r1_1 = _mm256_load_ps(arr1_1);
        __m256 r1_2 = _mm256_load_ps(arr1_2);
        __m256 r2 = _mm256_load_ps(arr2);

        __m256 re1_0 = _mm256_mul_ps(r1_0, r2);
        __m256 re1_1 = _mm256_mul_ps(r1_1, r2);
        __m256 re1_2 = _mm256_mul_ps(r1_2, r2);

        ALIGNED float res[8] = {0};
        ALIGNED float res1[8] = {0};
        ALIGNED float res2[8] = {0};

        _mm256_store_ps(res, re1_0);
        _mm256_store_ps(res1, re1_1);
        _mm256_store_ps(res2, re1_2);

        ret[0] = res[0] + res[1] + res[2];
        ret[1] = res[4] + res[5] + res[6];
        ret[2] = res1[0] + res1[1] + res1[2];
        ret[3] = res1[4] + res1[5] + res1[6];
        ret[4] = res2[0] + res2[1] + res2[2];
        ret[5] = res2[4] + res2[5] + res2[6];
        return ret;
    }

    inline static Eigen::Matrix<float, 4, 4>
    multiply4_4_by_4_4(Eigen::Matrix<float, 4, 4> &first, Eigen::Matrix<float, 4, 4> &sec) {
        ALIGNED float firstArr[8] = {
                first(0, 0), first(0, 1), first(0, 2), first(0, 3),
                first(0, 0), first(0, 1), first(0, 2), first(0, 3)
        };
        ALIGNED float secArr[8] = {
                first(1, 0), first(1, 1), first(1, 2), first(1, 3),
                first(1, 0), first(1, 1), first(1, 2), first(1, 3)
        };
        ALIGNED float thirdArr[8] = {
                first(2, 0), first(2, 1), first(2, 2), first(2, 3),
                first(2, 0), first(2, 1), first(2, 2), first(2, 3)
        };
        ALIGNED float fourth[8] = {
                first(3, 0), first(3, 1), first(3, 2), first(3, 3),
                first(3, 0), first(3, 1), first(3, 2), first(3, 3)
        };
        ALIGNED float Arr12[8] = {
                sec(0, 0), sec(1, 0), sec(2, 0), sec(3, 0),
                sec(0, 1), sec(1, 1), sec(2, 1), sec(3, 1)
        };

        ALIGNED float Arr34[8] = {
                sec(0, 2), sec(1, 2), sec(2, 2), sec(3, 2),
                sec(0, 3), sec(1, 3), sec(2, 3), sec(3, 3)
        };
        ALIGNED float res[8] = {0};
        Eigen::Matrix<float, 4, 4> ret;

        __m256 arr1 = _mm256_load_ps(firstArr);
        __m256 arr2 = _mm256_load_ps(Arr12);
        __m256 r = _mm256_mul_ps(arr1, arr2);

        _mm256_store_ps(res, r);

        ret(0, 0) = res[0] + res[1] + res[2] + res[3];
        ret(0, 1) = res[4] + res[5] + res[6] + res[7];

        arr1 = _mm256_load_ps(secArr);
        r = _mm256_mul_ps(arr1, arr2);
        _mm256_store_ps(res, r);

        ret(1, 0) = res[0] + res[1] + res[2] + res[3];
        ret(1, 1) = res[4] + res[5] + res[6] + res[7];

        arr1 = _mm256_load_ps(thirdArr);
        r = _mm256_mul_ps(arr1, arr2);
        _mm256_store_ps(res, r);

        ret(2, 0) = res[0] + res[1] + res[2] + res[3];
        ret(2, 1) = res[4] + res[5] + res[6] + res[7];

        arr1 = _mm256_load_ps(fourth);
        r = _mm256_mul_ps(arr1, arr2);
        _mm256_store_ps(res, r);

        ret(3, 0) = res[0] + res[1] + res[2] + res[3];
        ret(3, 1) = res[4] + res[5] + res[6] + res[7];

        arr2 = _mm256_load_ps(Arr34);
        r = _mm256_mul_ps(arr1, arr2);

        _mm256_store_ps(res, r);

        ret(3, 2) = res[0] + res[1] + res[2] + res[3];
        ret(3, 3) = res[4] + res[5] + res[6] + res[7];

        arr1 = _mm256_load_ps(thirdArr);
        r = _mm256_mul_ps(arr1, arr2);

        _mm256_store_ps(res, r);

        ret(2, 2) = res[0] + res[1] + res[2] + res[3];
        ret(2, 3) = res[4] + res[5] + res[6] + res[7];

        arr1 = _mm256_load_ps(secArr);
        r = _mm256_mul_ps(arr1, arr2);

        _mm256_store_ps(res, r);

        ret(1, 2) = res[0] + res[1] + res[2] + res[3];
        ret(1, 3) = res[4] + res[5] + res[6] + res[7];

        arr1 = _mm256_load_ps(firstArr);
        r = _mm256_mul_ps(arr1, arr2);

        _mm256_store_ps(res, r);

        ret(0, 2) = res[0] + res[1] + res[2] + res[3];
        ret(0, 3) = res[4] + res[5] + res[6] + res[7];

        return ret;
    }

    inline static Eigen::Vector3f
    get_global_posterior_vert(Eigen::Matrix<float, 4, 4> &global_pose_z_minus_1, Eigen::Vector3f &local_vert) {

        return local_vert_2_global_vert(global_pose_z_minus_1, local_vert);
    }

    static Tracking* createTrackingTool(TrackingMethod t_method = TrackingMethod::POINT_PLANE);

public:
    TrackingMethod method = TrackingMethod::POINT_PLANE;
#ifdef USE_AVX2
    USE_SIMD simd_ = USE_SIMD::AVX2;
#elif USE_AVX
    USE_SIMD simd_=USE_SIMD::AVX;
#elif USE_SSE4_2
    USE_SIMD simd_=USE_SIMD::SSE4_2;
#endif

public:
    Tracking() {};

    virtual ~Tracking() {};

    virtual bool init()=0 ;

    virtual Eigen::Matrix<float,4,4> iterationForPose() =0;

    virtual void setImagePairs(cv::Mat &rgb, cv::Mat &depth, double &timestamp) =0;

    inline virtual Eigen::Matrix<float, 4, 4> getPoseEstimation() =0;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


class PointPlaneTracking : public Tracking {
public:
    PointPlaneTracking(){};

    ~PointPlaneTracking(){};

public:
    bool init() override;

    Eigen::Matrix<float,4,4> iterationForPose() override;

    void setImagePairs(cv::Mat &rgb, cv::Mat &depth, double &timestamp) override;

    inline Eigen::Matrix<float, 4, 4> getPoseEstimation() override { return pose_posterior; }
};

#endif //FUSION_TRACKING_H

