#include "localmapping.h"

using namespace std;
using namespace cv;

LocalMapping::LocalMapping()
{
    cout<<"Construct Local Mapping "<<endl;
    
    this->mFastDetector = cv::FastFeatureDetector::create();

    this->mvNewFrames.reserve(500);
    this->mvvNewLabels.reserve(500);
    this->mvvTable.reserve(100);
}
void LocalMapping::Run()
{







}

void LocalMapping::AddObserved(const cv::Mat& img, const cv::Mat& K, const cv::Mat& Tcw, const std::vector<cv::Point2f>& vVertexes)
{
    Frame frame(img, K, this->mFastDetector);

    this->mvNewFrames.push_back(frame);

    //mvvNewLabels



}
