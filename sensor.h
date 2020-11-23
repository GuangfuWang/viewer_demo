//
// Created by root on 2020/11/10.
//

#ifndef TSFUSION_SENSOR_H
#define TSFUSION_SENSOR_H

#include <opencv2/opencv.hpp>
#include <libfreenect_registration.h>
#include "libfreenect.hpp"
#include "Sem.h"
#include "scope_mutex.h"
#include "thread.h"

typedef struct _freenect_context freenect_context;
typedef struct _freenect_device freenect_device;

class FreenectDevice : public Thread {
public:
    FreenectDevice(freenect_context * ctx, int index, bool color = true, bool registered = true) :
            index_(index),
            color_(color),
            registered_(registered),
            ctx_(ctx),
            device_(0),
            depthFocal_(0.0f)
    {
        assert(ctx_ != 0);
    }

    virtual ~FreenectDevice()
    {
        this->join(true);
        if(device_ && freenect_close_device(device_) < 0){}
    }

    const std::string & getSerial() const {return serial_;}

    bool init()
    {
        if(device_)
        {
            this->join(true);
            freenect_close_device(device_);
            device_ = 0;
        }
        serial_.clear();
        std::vector<std::string> deviceSerials;
        freenect_device_attributes* attr_list;
        freenect_device_attributes* item;
        freenect_list_device_attributes(ctx_, &attr_list);
        for (item = attr_list; item != NULL; item = item->next) {
            deviceSerials.push_back(std::string(item->camera_serial));
        }
        freenect_free_device_attributes(attr_list);

        if(freenect_open_device(ctx_, &device_, index_) < 0)
        {
            printf("FreenectDevice: Cannot open Kinect");
            return false;
        }
        if(index_ >= 0 && index_ < (int)deviceSerials.size())
        {
            serial_ = deviceSerials[index_];
        }
        else
        {
            printf("Could not get serial for index %d", index_);
        }
        printf("color=%d registered=%d", color_?1:0, registered_?1:0);
        freenect_set_user(device_, this);
        freenect_frame_mode videoMode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, color_?FREENECT_VIDEO_RGB:FREENECT_VIDEO_IR_8BIT);
        freenect_frame_mode depthMode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, color_ && registered_?FREENECT_DEPTH_REGISTERED:FREENECT_DEPTH_MM);
        if(!videoMode.is_valid)
        {
            printf("Freenect: video mode selected not valid!");
            return false;
        }
        if(!depthMode.is_valid)
        {
            printf("Freenect: depth mode selected not valid!");
            return false;
        }
        assert(videoMode.data_bits_per_pixel == 8 || videoMode.data_bits_per_pixel == 24);
        assert(depthMode.data_bits_per_pixel == 16);
        freenect_set_video_mode(device_, videoMode);
        freenect_set_depth_mode(device_, depthMode);
        rgbIrBuffer_ = cv::Mat(cv::Size(videoMode.width,videoMode.height), color_?CV_8UC3:CV_8UC1);
        depthBuffer_ = cv::Mat(cv::Size(depthMode.width,depthMode.height), CV_16UC1);
        freenect_set_depth_buffer(device_, depthBuffer_.data);
        freenect_set_video_buffer(device_, rgbIrBuffer_.data);
        freenect_set_depth_callback(device_, freenect_depth_callback);
        freenect_set_video_callback(device_, freenect_video_callback);

        float rgb_focal_length_sxga = 1050.0f;
        float width_sxga = 1280.0f;
        float width = freenect_get_current_depth_mode(device_).width;
        float scale = width / width_sxga;
        if(color_ && registered_)
        {
            depthFocal_ =  rgb_focal_length_sxga * scale;
        }
        else
        {
            freenect_registration reg = freenect_copy_registration(device_);
            float depth_focal_length_sxga = reg.zero_plane_info.reference_distance / reg.zero_plane_info.reference_pixel_size;
            freenect_destroy_registration(&reg);

            depthFocal_ =  depth_focal_length_sxga * scale;
        }
        printf("FreenectDevice: Depth focal = %f", depthFocal_);
        return true;
    }
    float getDepthFocal() const {return depthFocal_;}
    void getData(cv::Mat & rgb, cv::Mat & depth)
    {
        if(this->isRunning()){
            if(!dataReady_.acquire(1, 5000))
            {
                printf("Not received any frames since 5 seconds, try to restart the camera again.");
            }
            else
            {
                ScopeMutex s(dataMutex_);
                rgb = rgbIrLastFrame_;
                depth = depthLastFrame_;
                rgbIrLastFrame_ = cv::Mat();
                depthLastFrame_= cv::Mat();
            }
        }
    }

    void getAccelerometerValues(double & x, double & y, double & z)
    {
        freenect_update_tilt_state(device_);
        freenect_raw_tilt_state* state = freenect_get_tilt_state(device_);
        freenect_get_mks_accel(state, &x,&y,&z);
    }

private:
    // Do not call directly even in child
    void VideoCallback(void* rgb)
    {
        assert(rgbIrBuffer_.data == rgb);
        ScopeMutex s(dataMutex_);
        bool notify = rgbIrLastFrame_.empty();
        if(color_)
        {
            cv::cvtColor(rgbIrBuffer_, rgbIrLastFrame_, cv::COLOR_RGB2BGR);
        }
        else // IrDepth
        {
            rgbIrLastFrame_ = rgbIrBuffer_.clone();
        }
        if(!depthLastFrame_.empty() && notify)
        {
            dataReady_.release();
        }
    }

    // Do not call directly even in child
    void DepthCallback(void* depth)
    {
        assert(depthBuffer_.data == depth);
        ScopeMutex s(dataMutex_);
        bool notify = depthLastFrame_.empty();
        depthLastFrame_ = depthBuffer_.clone();
        if(!rgbIrLastFrame_.empty() && notify)
        {
            dataReady_.release();
        }
    }

    void startVideo() {
        if(device_ && freenect_start_video(device_) < 0)
            printf("Cannot start RGB callback");
    }
    void stopVideo() {
        if(device_ && freenect_stop_video(device_) < 0)
            printf("Cannot stop RGB callback");
    }
    void startDepth() {
        if(device_ && freenect_start_depth(device_) < 0)
            printf("Cannot start depth callback");
    }
    void stopDepth() {
        if(device_ && freenect_stop_depth(device_) < 0)
            printf("Cannot stop depth callback");
    }

    virtual void mainLoopBegin()
    {
        if(device_) freenect_set_led(device_, LED_RED);
        this->startDepth();
        this->startVideo();
    }

    virtual void mainLoop()
    {
        timeval t;
        t.tv_sec = 0;
        t.tv_usec = 10000;
        if(freenect_process_events_timeout(ctx_, &t) < 0)
        {
            printf("FreenectDevice: Cannot process freenect events");
            this->kill();
        }
    }

    virtual void mainLoopEnd()
    {
        if(device_) freenect_set_led(device_, LED_GREEN);
        this->stopDepth();
        this->stopVideo();
        dataReady_.release();
    }

    static void freenect_depth_callback(freenect_device *dev, void *depth, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->DepthCallback(depth);
    }
    static void freenect_video_callback(freenect_device *dev, void *video, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->VideoCallback(video);
    }

    //noncopyable
    FreenectDevice( const FreenectDevice& );
    const FreenectDevice& operator=( const FreenectDevice& );

private:
    int index_;
    bool color_;
    bool registered_;
    std::string serial_;
    freenect_context * ctx_;
    freenect_device * device_;
    cv::Mat depthBuffer_;
    cv::Mat rgbIrBuffer_;
    Mutex dataMutex_;
    cv::Mat depthLastFrame_;
    cv::Mat rgbIrLastFrame_;
    float depthFocal_;
    Semaphore dataReady_;
};
enum class device{
    KINECT_V1=0,
    KINECT_V2=1,
    PRIME_SENSE=2,
    UNKNOWN=3
};

class sensor{
public:
    static sensor* createSensor(device dev=device::KINECT_V1);
public:
    virtual bool getData(cv::Mat& rgb,cv::Mat& dep)=0;
protected:
    cv::Mat s_rgb;
    cv::Mat s_depth;

public:
    sensor(){}
    virtual ~sensor(){}
};

class kinect1: public sensor{
private:
    _freenect_context * ctx_;
    FreenectDevice * freenectDevice_;
public:
    bool getData(cv::Mat& rgb,cv::Mat& dep)override ;

public:
    kinect1();
    ~kinect1(){
        if(freenectDevice_)
        {
            freenectDevice_->join(true);
            delete freenectDevice_;
            freenectDevice_ = 0;
        }
        if(ctx_)
        {
            if(freenect_shutdown(ctx_) < 0){}
        }
    }
    bool init();
};



#endif //TSFUSION_SENSOR_H
