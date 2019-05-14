#pragma once
#ifndef LABEL_H
#define LABEL_H
#include "frame.h"
#include <opencv2/opencv.hpp>

class Label
{
public:
    Label(const Frame& frame, const std::vector<cv::Point2f>& vVertexes, const int localId);

public:
    int mLocalId;
    Frame mFrame;
    std::vector<cv::Point2f> mvVertexes;
    
};

#endif //LABEL_H