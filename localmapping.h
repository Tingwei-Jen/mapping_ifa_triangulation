#pragma once
#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H
#include "frame.h"
#include "label.h"
#include "map.h"

struct Detection
{
    cv::Mat img;
    std::vector<std::vector<cv::Point2f>> vVertexess;
}; //From DL structure

class LocalMapping
{
public:
    LocalMapping(){}

    LocalMapping(cv::Mat K, Map* map);

    //main function
    void Run();

    //add from DL
    void UpdateReference(const Detection& ref);
    
    //add from VEPP
    void AddNewImg(const cv::Mat& img, const cv::Mat& Tcw = cv::Mat::eye(4,4, CV_32F));

    Map* GepMap(){ return mMap; }
 
private: 
    //initial
    void Initialization();

    //check is there any new frame in list
    bool CheckNewFrames();

    //tracker, generate labels
    void ProcessNewFrame();

    //generate sign, if no more label
    void CreateNewSign();

    //Two tracker
    void TrackReference();
    void TrackLastFrame();

    //IFA
    void InterFrameAssociation(Label* label);

    //create sign's 3D location
    bool Triangulation(const std::vector<cv::Mat>& Tcws, const std::vector<cv::Point3f>& ptsCam, cv::Point3f& x3Dp);

    void PrintConnection();

private:

    // Mapping states
    enum eMappingState{
        NOT_INITIALIZED,
        OK,
    };

    eMappingState mState;

    cv::Mat mK; 

    cv::Ptr<cv::FastFeatureDetector> mFastDetector;

    Detection mDetectionRef;
    bool mbNewDetectionRef;

    std::list<Frame*> mlpNewFrames;
    Frame* mpCurrentFrame;
    Frame* mpLastFrame;

    Map* mMap;

private:
    const float mDistOfLabelsTH = 5.0;


};

#endif //LOCALMAPPING_H