#include "label.h"
#include <iostream>

Label::Label(Frame* frame, const std::vector<cv::Point2f>& vVertexes, const std::vector<cv::Point2f>& vKeyPoints, const int localId)
:mLocalId(localId)
{
    std::cout<<"Construct Label Local Id: "<<mLocalId<<std::endl;
    
    this->mpFrame = frame;

    this->mCenterPx = cv::Point2f(0,0);
    for(int i=0; i<vVertexes.size(); i++)
    {
        this->mvVertexes.push_back(vVertexes[i]);
        this->mCenterPx += vVertexes[i];
    }
    this->mCenterPx /= (int)vVertexes.size();
        
    for(int i=0; i<vKeyPoints.size(); i++)
        this->mvKeyPoints.push_back(vKeyPoints[i]);
}