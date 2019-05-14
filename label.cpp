#include "label.h"
#include <iostream>

Label::Label(const Frame& frame, const std::vector<cv::Point2f>& vVertexes, const int localId):mLocalId(localId)
{
    std::cout<<"Construct Label Local Id: "<<mLocalId<<std::endl;
    
    this->mFrame = Frame(frame);

    for(int i=0; i<vVertexes.size(); i++)
        this->mvVertexes.push_back(vVertexes[i]);
}