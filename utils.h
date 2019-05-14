#pragma once
#ifndef UTILS_H
#define UTILS_H
#include <opencv2/opencv.hpp>
#include <string>  

class Utils
{
public:
    Utils();
    cv::Mat GetMyntK(){ return mMyntK; }
    void ReadImgs(const std::string& pattern, std::vector<cv::Mat>& imgs);
    void ReadCameraPose(const std::string csv_path, std::vector<cv::Mat>& Tcws);

private:
    cv::Mat Quaternion2RotM(float x, float y, float z, float w);

private:
    cv::Mat mMyntK;
};

#endif //UTILS_H