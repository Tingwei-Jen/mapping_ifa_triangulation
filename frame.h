#pragma once
#ifndef FRAME_H
#define FRAME_H
#include <opencv2/opencv.hpp>

class Frame
{
public:
    Frame();

    //Copy
    Frame(const Frame& frame);

    // Constructor for Monocular cameras.
	Frame(const cv::Mat& img, const cv::Mat &K, const cv::Ptr<cv::FastFeatureDetector>& detector);

    void SetPose(const cv::Mat& Tcw);

    //Pose
	cv::Mat GetPose(){ return mTcw.clone(); }
    cv::Mat GetCameraCenter(){ return mOw.clone(); }
    cv::Mat GetRotation(){ return mRcw.clone(); }
    cv::Mat GetTranslation(){ return mtcw.clone(); }
    cv::Mat GetRotationInverse(){ return mRwc.clone(); }

    //Coordinate Transfer
    cv::Point2f Cam2Px(const cv::Point3f& pCam);
    cv::Point3f Px2Cam(const cv::Point2f& px);
    cv::Point3f World2Cam(const cv::Point3f& pWorld);
    cv::Point3f Cam2World(const cv::Point3f& pCam);

    std::vector<int> GetFeaturesInArea(const float& x, const float& y, const float& rx, const float& ry);

public:
    static int frameCounter;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static int width;
    static int height;
    static bool mInit;
    
    int mId;
    cv::Mat mImg;
    cv::Mat mK;
    std::vector<cv::Point2f> mvKps; 

private:
    cv::Ptr<cv::FastFeatureDetector> mDetector;
	cv::Mat mTcw;                                          ///< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵
    cv::Mat mRcw;                                          ///< Rotation from world to camera
    cv::Mat mRwc;                                          ///< Rotation from camera to world
    cv::Mat mtcw;                                          ///< Translation from world to camera   
    cv::Mat mOw;                                           ///< mtwc,Translation from camera to world

};


#endif //FRAME_H