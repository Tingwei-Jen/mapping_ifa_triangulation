#include "localmapping.h"

using namespace std;
using namespace cv;

LocalMapping::LocalMapping(cv::Mat K, Map* map)
:mK(K), mFastDetector(cv::FastFeatureDetector::create()), mbNewDetectionRef(false), mState(NOT_INITIALIZED)
{
    cout<<"Construct Local Mapping "<<endl;
    this->mLableTable.reserve(100);
}

void LocalMapping::Run()
{
    //while(1)
    //{
        if(this->mState == NOT_INITIALIZED) 
        {
            Initialization();
        }
        else
        {
            if(CheckNewFrames())
            {
                ProcessNewFrame();








            }
            else
            {
                //sleep
            }


        }







    //}
}


//add from DL
void LocalMapping::UpdateReference(const Detection& ref)
{
    this->mDetectionRef = ref;
    this->mbNewDetectionRef = true;
}
    
//add from VEPP
void LocalMapping::AddNewImg(const cv::Mat& img, const cv::Mat& Tcw)
{
    Frame* frame = new Frame(img, this->mK, this->mFastDetector);
    frame->SetPose(Tcw);

    this->mplNewFrames.push_back(frame);
}

/*
private
*/
void LocalMapping::Initialization()
{
    this->mpLastFrame = this->mplNewFrames.front();
    this->mplNewFrames.pop_front();
    this->mState = OK;
    cout<<"Initialization!! "<<endl;
}

bool LocalMapping::CheckNewFrames()
{
    return (!this->mplNewFrames.empty());
}

void LocalMapping::ProcessNewFrame()
{
    this->mpCurrentFrame = this->mplNewFrames.front();
    this->mplNewFrames.pop_front();

    if(this->mbNewDetectionRef) //track reference
    {
        cout<<"TrackReference!! "<<endl;
        TrackReference();





        this->mpLastFrame = this->mpCurrentFrame;
        this->mbNewDetectionRef = false;
    }
    else //track last frame
    {
        cout<<"TrackLastFrame!! "<<endl;
        TrackLastFrame();






        this->mpLastFrame = this->mpCurrentFrame;
    }


}



void LocalMapping::TrackReference()
{
    Mat imgRef = mDetectionRef.img;
    Mat imgCurrent = this->mpCurrentFrame->mImg;

    //0. Get feature points in Reference image
    vector<KeyPoint> vKeyPoints;
    mFastDetector->detect(imgRef, vKeyPoints);
    
    vector<Point2f> vRefKps;
    cv::KeyPoint::convert(vKeyPoints, vRefKps);

    int nDetects = mDetectionRef.vVertexess.size();

    for(int i=0; i<nDetects; i++)
    {
        //1. find kps in detection sign
        vector<Point2f> vRefKpsInArea;

        if((int)mDetectionRef.vVertexess[i].size() == 4) //rectangle
        {
            Point p0 = mDetectionRef.vVertexess[i][0];
            Point p1 = mDetectionRef.vVertexess[i][1];
            Point p2 = mDetectionRef.vVertexess[i][2];
            Point p3 = mDetectionRef.vVertexess[i][3];

            int minX = std::max(p0.x, p3.x);
            int maxX = std::min(p1.x, p2.x);
            int minY = std::max(p0.y, p1.y);
            int maxY = std::min(p2.y, p3.y);

            for(int j=0; j<vRefKps.size(); j++)
            {   
                if(vRefKps[j].x>minX && vRefKps[j].x<maxX && vRefKps[j].y>minY && vRefKps[j].y<maxY)
                {
                    vRefKpsInArea.push_back(vRefKps[j]);
                }     
            }
        }
        else //triangle
        {
            //TODO
        }

        //2. feature matching by optical flow
        vector<Point2f> vCurrentKps;
        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(imgRef,  imgCurrent, vRefKpsInArea, vCurrentKps, status, err);

        //3. homography
        Mat inlier_mask, homography;
        const double ransac_thresh = 2.5f; 
        homography = findHomography(vRefKpsInArea, vCurrentKps, CV_RANSAC, ransac_thresh, inlier_mask);
        
        //4. Get new vertex
        vector<Point2f> vNewVertexes;
        perspectiveTransform(mDetectionRef.vVertexess[i], vNewVertexes, homography);

        // Out of boundry
        if(vNewVertexes[0].x<0 || vNewVertexes[0].y<0 || vNewVertexes[0].x>this->mpCurrentFrame->width || vNewVertexes[0].y>this->mpCurrentFrame->height
        || vNewVertexes[1].x<0 || vNewVertexes[1].y<0 || vNewVertexes[1].x>this->mpCurrentFrame->width || vNewVertexes[1].y>this->mpCurrentFrame->height
        || vNewVertexes[2].x<0 || vNewVertexes[2].y<0 || vNewVertexes[2].x>this->mpCurrentFrame->width || vNewVertexes[2].y>this->mpCurrentFrame->height)
            continue;

        if(vNewVertexes.size()==4)
        {
            if(vNewVertexes[3].x<0 || vNewVertexes[3].y<0 || vNewVertexes[3].x>this->mpCurrentFrame->width || vNewVertexes[3].y>this->mpCurrentFrame->height)
                continue;
        }

        //5. Create label
        Label* label = new Label(this->mpCurrentFrame, vNewVertexes, vCurrentKps, i);
        
        //6. Add into frame
        this->mpCurrentFrame->AddLabels(label);

        //7. IFA, Put into table
        InterFrameAssociation(label);

    }
}

void LocalMapping::TrackLastFrame()
{



}

//IFA between last label in table and current label, then put label in the right place
//only call in TrackReference()
void LocalMapping::InterFrameAssociation(Label* label)
{
    Mat imgRef = mDetectionRef.img;
    Mat imgLast = this->mpLastFrame->mImg;
    Mat imgCurrent = this->mpCurrentFrame->mImg;

    cout<<"IFA"<<endl;

    for(int i=0; i<this->mLableTable.size(); i++)
    {
        if(this->mLableTable[i].size()!=0) //Do IFA
        {
            //get last label
            Label* labelLast = this->mLableTable[i][this->mLableTable[i].size()-1];  //last one

            //feature matching by optical flow
            vector<Point2f> vCurrentKpsPred;
            vector<uchar> status;
            vector<float> err;
            calcOpticalFlowPyrLK(imgLast, imgCurrent, labelLast->mvKeyPoints, vCurrentKpsPred, status, err);

            //find homography
            Mat inlier_mask, homography;
            const double ransac_thresh = 2.5f; 
            homography = findHomography(labelLast->mvKeyPoints, vCurrentKpsPred, CV_RANSAC, ransac_thresh, inlier_mask);

            //project label center in last to current
            vector<Point2f> vCenterPxLast;
            vCenterPxLast.push_back(labelLast->mCenterPx);

            vector<Point2f> vNewCenterPxPred;
            perspectiveTransform(vCenterPxLast, vNewCenterPxPred, homography);

            
            //dist between center
            float dist = norm(vNewCenterPxPred[0]-label->mCenterPx);

            if(dist > this->mDistOfLabelsTH)
                continue;

            //Same label, put into Table
            this->mLableTable[i].push_back(label);
        }
    }

    //New sign
    std::vector<Label*> vNewObservedSign;
    vNewObservedSign.push_back(label);

    if(this->mVacantList.empty())
    {
        this->mLableTable.push_back(vNewObservedSign);
    }
    else
    {
        int idx = this->mVacantList.front();
        this->mVacantList.pop_front();
        this->mLableTable[idx] = vNewObservedSign;
    }

    // imshow("imgRef",imgRef);
    // imshow("imgLast",imgLast);
    // imshow("imgCurrent",imgCurrent);

    // waitKey(0);
}