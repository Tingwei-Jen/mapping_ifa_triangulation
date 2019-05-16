#include "localmapping.h"

using namespace std;
using namespace cv;

LocalMapping::LocalMapping(cv::Mat K, Map* map)
:mK(K), mFastDetector(cv::FastFeatureDetector::create()), mbNewDetectionRef(false), mState(NOT_INITIALIZED), mMap(map)
{
    cout<<"Construct Local Mapping "<<endl;
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

                CreateNewSign();

                this->mpLastFrame = this->mpCurrentFrame;
            }
            else
            {
                //sleep
            }



            PrintConnection();


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

    this->mlpNewFrames.push_back(frame);
}

/*
private
*/
void LocalMapping::Initialization()
{
    this->mpLastFrame = this->mlpNewFrames.front();
    this->mlpNewFrames.pop_front();
    this->mState = OK;
    cout<<"Initialization!! "<<endl;
}

bool LocalMapping::CheckNewFrames()
{
    return (!this->mlpNewFrames.empty());
}

void LocalMapping::ProcessNewFrame()
{
    this->mpCurrentFrame = this->mlpNewFrames.front();
    this->mlpNewFrames.pop_front();

    if(this->mbNewDetectionRef) //track reference
    {
        cout<<"TrackReference!! "<<endl;
        TrackReference();

        this->mbNewDetectionRef = false;
    }
    else //track last frame
    {
        cout<<"TrackLastFrame!! "<<endl;
        TrackLastFrame();
    }
}

void LocalMapping::CreateNewSign()
{
    int nDetects = this->mpLastFrame->mvpLabels.size();

    for(int i=0; i<nDetects; i++)
    {
        Label* labelLast = this->mpLastFrame->mvpLabels[i];

        if(!labelLast->next) //means that couldn't see the sign any more.
        {
            //1. Gather Labels
            vector<Label*> vpLabels;
            Label* pre = labelLast;
            
            while(pre)
            {
                vpLabels.push_back(pre);               
                pre = pre->previous;
            }

            if(vpLabels.size()<2)  //cannot do triangulation
                continue;

            cout<<"create sign  ";
            for(int j=0; j<vpLabels.size(); j++)
                cout<<vpLabels[j]->mLocalId<<"  ";
            cout<<endl;

            //2. Triangulation
            int nVertex = vpLabels[0]->mvVertexes.size();

            vector<Point3f> vVertexes3D;

            for(int j=0; j<nVertex; j++)
            {
                vector<cv::Mat> Tcws;
                vector<Point3f> ptsCam;
                
                for(int k=0; k<vpLabels.size(); k++)
                {
                    Tcws.push_back(vpLabels[k]->mpFrame->GetPose());
                
                    Point2f px = vpLabels[k]->mvVertexes[j];
                    ptsCam.push_back(vpLabels[k]->mpFrame->Px2Cam(px));
                }

                cv::Point3f x3Dp;
                Triangulation(Tcws, ptsCam, x3Dp);

                vVertexes3D.push_back(x3Dp);
            }

            //3. Create Sign
            Sign* sign = new Sign(vpLabels);
            sign->SetVertexes3D(vVertexes3D);

            this->mMap->AddSign(sign);
        }
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

        //7. IFA, Update connection
        InterFrameAssociation(label);

    }
}

void LocalMapping::TrackLastFrame()
{

    Mat imgLast = this->mpLastFrame->mImg;
    Mat imgCurrent = this->mpCurrentFrame->mImg;

    int nDetects = this->mpLastFrame->mvpLabels.size();

    for(int i=0; i<nDetects; i++)
    {
        //1. get label in last frame
        Label* labelLast = this->mpLastFrame->mvpLabels[i];

        //2. feature matching by optical flow
        vector<Point2f> vCurrentKps;
        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(imgLast, imgCurrent, labelLast->mvKeyPoints, vCurrentKps, status, err);

        //3. homography
        Mat inlier_mask, homography;
        const double ransac_thresh = 2.5f; 
        homography = findHomography(labelLast->mvKeyPoints, vCurrentKps, CV_RANSAC, ransac_thresh, inlier_mask);
        
        //4. Get new vertex
        vector<Point2f> vNewVertexes;
        perspectiveTransform(labelLast->mvVertexes, vNewVertexes, homography);

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

        //5. Create label, same id with label in last frame
        Label* label = new Label(this->mpCurrentFrame, vNewVertexes, vCurrentKps, labelLast->mLocalId);
        
        //update connection
        label->previous = labelLast;
        labelLast->next = label;

        //6. Add into frame
        this->mpCurrentFrame->AddLabels(label);
    }
}

//IFA between last label in table and current label, if the same one, update connection
//only call in TrackReference()
void LocalMapping::InterFrameAssociation(Label* label)
{
    Mat imgLast = this->mpLastFrame->mImg;
    Mat imgCurrent = this->mpCurrentFrame->mImg;

    int nDetects = this->mpLastFrame->mvpLabels.size();

    for(int i=0; i<nDetects; i++)
    {
        //1. get label in last frame
        Label* labelLast = this->mpLastFrame->mvpLabels[i];

        //2. feature matching by optical flow
        vector<Point2f> vCurrentKpsPred;
        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(imgLast, imgCurrent, labelLast->mvKeyPoints, vCurrentKpsPred, status, err);

        //3. find homography
        Mat inlier_mask, homography;
        const double ransac_thresh = 2.5f; 
        homography = findHomography(labelLast->mvKeyPoints, vCurrentKpsPred, CV_RANSAC, ransac_thresh, inlier_mask);

        //4. project label center in last to current
        vector<Point2f> vCenterPxLast;
        vCenterPxLast.push_back(labelLast->mCenterPx);

        vector<Point2f> vNewCenterPxPred;
        perspectiveTransform(vCenterPxLast, vNewCenterPxPred, homography);
    
        //5. dist between center
        float dist = norm(vNewCenterPxPred[0]-label->mCenterPx);

        if(dist > this->mDistOfLabelsTH)
            continue;

        //Same label, update connection
        label->previous = labelLast;
        labelLast->next = label;
    }
}

// |xp2  - p0 |     |0|
// |yp2  - p1 | X = |0| ===> AX = 0
// |x'p2'- p0'|     |0|
// |y'p2'- p1'|     |0|
bool LocalMapping::Triangulation(const std::vector<cv::Mat>& Tcws, const std::vector<cv::Point3f>& ptsCam, cv::Point3f& x3Dp)
{
   int n_frames = Tcws.size();

    Mat A(n_frames*2, 4, CV_32F);
   
    for(int i=0; i<n_frames; i++)
    {
        A.row(i*2) = ptsCam[i].x*Tcws[i].row(2)-Tcws[i].row(0);
        A.row(i*2+1) = ptsCam[i].y*Tcws[i].row(2)-Tcws[i].row(1);
    }
    
    Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    Mat x3D = vt.row(3).t();

    if(x3D.at<float>(3) == 0)
        return false;
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);  

    // check triangulation in front of camera.
    for(int i=0; i<n_frames; i++)
    {
        Mat Rcw, tcw;
        Tcws[i].rowRange(0,3).colRange(0,3).copyTo(Rcw);
        Tcws[i].rowRange(0,3).col(3).copyTo(tcw);

        Mat x3Dt = x3D.t();

        float z = Rcw.row(2).dot(x3Dt)+tcw.at<float>(2);
        if(z<=0)
            return false;
    }

    x3Dp = Point3f(x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2));
    return true;
}


void LocalMapping::PrintConnection()
{
    int nDetects = this->mpCurrentFrame->mvpLabels.size();

    for(int i=0; i<nDetects; i++)
    {
        Label* label = this->mpCurrentFrame->mvpLabels[i];

        Label* pre = label;
        while(pre)
        {
            cout<<pre->mLocalId<<"  ";

            Mat img; 
            cv::cvtColor(pre->mpFrame->mImg.clone(), img, cv::COLOR_GRAY2BGR);  
            
            for(int j=0; j<pre->mvVertexes.size(); j++)
                cv::circle(img, pre->mvVertexes[j], 4, cv::Scalar(255,0,0), -1);
            
            imshow("img", img);
            waitKey(0);

            pre = pre->previous;
        }
        cout<<endl;
    }
}