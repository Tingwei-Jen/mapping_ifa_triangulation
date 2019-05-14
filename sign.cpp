#include "sign.h"
#include <iostream>

int Sign::signCounter = 0;

Sign::Sign(const std::vector<Label>& vLabels):mId(signCounter++)
{
    std::cout<<"Construct Sign ID: "<<this->mId<<std::endl;

    for(int i=0; i<vLabels.size(); i++)
        this->mvLabels.push_back(vLabels[i]);
}

void Sign::SetVertexes(const std::vector<cv::Point3f>& vVertexes)
{
    this->mvVertexes.clear();
    for(int i=0; i<vVertexes.size(); i++)
        this->mvVertexes.push_back(vVertexes[i]);
}

