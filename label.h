#pragma once
#ifndef LABEL_H
#define LABEL_H
#include "frame.h"
#include <opencv2/opencv.hpp>

class Frame;

class Label
{
public:
    Label(Frame* frame, const std::vector<cv::Point2f>& vVertexes, const std::vector<cv::Point2f>& vKeyPoints, const int localId);

public:
    int mLocalId;
    Frame* mpFrame;
    std::vector<cv::Point2f> mvVertexes;
    std::vector<cv::Point2f> mvKeyPoints;

    cv::Point2f mCenterPx;

    Label* previous;
    Label* next;
};

#endif //LABEL_H