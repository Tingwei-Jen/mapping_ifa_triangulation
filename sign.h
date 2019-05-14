#pragma once
#ifndef SIGN_H
#define SIGN_H
#include "label.h"
#include <opencv2/opencv.hpp>

class Sign
{
public:
    Sign(const std::vector<Label>& vLabels);
    void SetVertexes(const std::vector<cv::Point3f>& vVertexes);
    std::vector<cv::Point3f> GetVertexes(){ return mvVertexes; }

public:
    int mId;
    static int signCounter;
    std::vector<Label> mvLabels;

private:
    std::vector<cv::Point3f> mvVertexes;
    
};

#endif //SIGN_H