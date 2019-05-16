#include "sign.h"
#include <iostream>

int Sign::signCounter = 0;

Sign::Sign(std::vector<Label*> vpLabels):mId(signCounter++)
{
    std::cout<<"Construct Sign ID: "<<this->mId<<std::endl;

    for(int i=0; i<vpLabels.size(); i++)
        this->mvpLabels.push_back(vpLabels[i]);
}

void Sign::SetVertexes3D(const std::vector<cv::Point3f>& vVertexes3D)
{
    this->mvVertexes3D.clear();
    for(int i=0; i<vVertexes3D.size(); i++)
        this->mvVertexes3D.push_back(vVertexes3D[i]);
}

