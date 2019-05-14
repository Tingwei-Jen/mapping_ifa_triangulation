#pragma once
#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H
#include "frame.h"
#include "label.h"
 
class LocalMapping
{
public:
    LocalMapping();

    void Run();

    void AddObserved(const cv::Mat& img, const cv::Mat& K, const cv::Mat& Tcw, const std::vector<cv::Point2f>& vVertexes);

private:
    std::vector<Frame> mvNewFrames;                              //new frames waiting for processing
    std::vector<std::vector<Label>> mvvNewLabels;                //new labels in each frame waiting for processing
    
    std::vector<std::vector<Label>> mvvTable;                    

    cv::Ptr<cv::FastFeatureDetector> mFastDetector;
};

#endif //LOCALMAPPING_H