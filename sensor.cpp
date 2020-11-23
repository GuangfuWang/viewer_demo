////
//// Created by root on 2020/11/10.
////
#include "sensor.h"
#include <unistd.h>

sensor *sensor::createSensor(device dev) {
    if(dev==device::KINECT_V1) {
        kinect1 *ret = new kinect1();
        if(ret->init())
            return ret;
        printf("please plug in kinect device first...");
        return nullptr;
    }
    printf("unsupported sensor...");
    return nullptr;
}

bool kinect1::getData(cv::Mat& rgb,cv::Mat& dep) {
    freenectDevice_->getData(s_rgb,s_depth);
    rgb=s_rgb.clone();
    dep=s_depth.clone();
    return !(rgb.empty() || dep.empty());
}


kinect1::kinect1():ctx_(0),freenectDevice_(0){
    if(freenect_init(&ctx_, NULL) < 0) printf("Cannot initialize freenect library");
    // claim camera
    freenect_select_subdevices(ctx_, static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
}

bool kinect1::init() {
    if(freenectDevice_)
    {
        freenectDevice_->join(true);
        delete freenectDevice_;
        freenectDevice_ = 0;
    }

    if(ctx_ && freenect_num_devices(ctx_) > 0)
    {
        // look for calibration files
        bool hardwareRegistration = true;
        freenectDevice_ = new FreenectDevice(ctx_, 0, true, hardwareRegistration);
        if(freenectDevice_->init())
        {
            freenectDevice_->start();
            uSleep(3000);
            return true;
        }
        else
        {
            printf("CameraFreenect: Init failed!");
        }
        delete freenectDevice_;
        freenectDevice_ = 0;
    }
    else
    {
        printf("CameraFreenect: No devices connected!");
    }
    return false;
}




