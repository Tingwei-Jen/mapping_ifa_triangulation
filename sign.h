#pragma once
#ifndef SIGN_H
#define SIGN_H
#include "label.h"
#include <opencv2/opencv.hpp>

class Sign //3D object
{
public:
    Sign(std::vector<Label*> vpLabels);
    void SetVertexes3D(const std::vector<cv::Point3f>& vVertexes3D);
    std::vector<cv::Point3f> GetVertexes3D(){ return mvVertexes3D; }

public:
    int mId;
    static int signCounter;
    std::vector<Label*> mvpLabels;

private:
    std::vector<cv::Point3f> mvVertexes3D;
    
};

#endif //SIGN_H