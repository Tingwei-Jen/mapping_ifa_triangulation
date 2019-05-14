#include "frame.h"
#include "converter.h"

int Frame::frameCounter = 0;
float Frame::fx, Frame::fy, Frame::cx, Frame::cy;
int Frame::width, Frame::height;
bool Frame::mInit=true;

Frame::Frame()
{
   
}

Frame::Frame(const Frame& frame)
:mId(frame.mId), mImg(frame.mImg), mK(frame.mK), mvKps(frame.mvKps)
{
    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

// Initialization
Frame::Frame(const cv::Mat& img, const cv::Mat &K, const cv::Ptr<cv::FastFeatureDetector>& detector)
:mImg(img.clone()), mK(K.clone()), mDetector(detector)
{
    // Frame ID
	this->mId = frameCounter++;

    // Extract features
    std::vector<cv::KeyPoint> vKeyPoints;
    this->mDetector->detect(mImg, vKeyPoints);
    cv::KeyPoint::convert(vKeyPoints, this->mvKps);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mInit)
    {
        mInit = false;

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        width = img.cols;
        height = img.rows;
    }
}

void Frame::SetPose(const cv::Mat& Tcw)
{
	this->mTcw = Tcw.clone();   
    this->mRcw = this->mTcw.rowRange(0,3).colRange(0,3);
    this->mRwc = this->mRcw.t();
    this->mtcw = this->mTcw.rowRange(0,3).col(3);
    this->mOw = -this->mRcw.t()*this->mtcw;
}

cv::Point2f Frame::Cam2Px(const cv::Point3f& pCam)
{
    cv::Point2f px = cv::Point2f(pCam.x*fx/pCam.z + cx, pCam.y*fy/pCam.z + cy);
    return px;
}

cv::Point3f Frame::Px2Cam(const cv::Point2f& px)
{
    cv::Point3f pCam = cv::Point3f((px.x-cx)/fx, (px.y-cy)/fy, 1);
    return pCam;
}

cv::Point3f Frame::World2Cam(const cv::Point3f& pWorld)
{
    cv::Mat pCam = this->mRcw*Converter::toCvMat(pWorld) + this->mtcw;
    return Converter::toCvPoint3f(pCam);
}

cv::Point3f Frame::Cam2World(const cv::Point3f& pCam)
{
    cv::Mat pWorld = this->mRwc*Converter::toCvMat(pCam) + this->mOw;
    return Converter::toCvPoint3f(pWorld);
}

/**
 * @brief 找到在 以x,y为中心,半徑r的方形内的特征点
 * @param x        图像坐标u
 * @param y        图像坐标v
 * @param r        半徑
 * @return         满足条件的特征点的序号
 */
std::vector<int> Frame::GetFeaturesInArea(const float& x, const float& y, const float& rx, const float& ry)
{
    std::vector<int> vIndices;
    vIndices.reserve(this->mvKps.size());  

    int minX, minY, maxX, maxY;

    minX = std::max(0,(int)(x-rx));
    if(minX>=width)
        return vIndices;

    maxX = std::min(width-1,(int)(x+rx));
    if(maxX<0)
        return vIndices;

    minY = std::max(0,(int)(y-ry));
    if(minY>=height)
        return vIndices;

    maxY = std::min(height-1,(int)(y+ry));
    if(maxY<0)
        return vIndices;

    //find key points in the boundry
    for(int i=0; i<this->mvKps.size(); i++)
    {   
        if(this->mvKps[i].x>minX && this->mvKps[i].x<maxX && this->mvKps[i].y>minY && this->mvKps[i].y<minY)
            vIndices.push_back(i);
    }

}

