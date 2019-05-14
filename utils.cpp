#include "utils.h"
#include <iostream>

using namespace std;
using namespace cv;

Utils::Utils()
{
    cout<<"Construct Utils"<<endl;
    this->mMyntK = cv::Mat(3,3, CV_32F);

    mMyntK.at<float>(0,0) = 712.584655; mMyntK.at<float>(0,1) = 0.0;        mMyntK.at<float>(0,2) = 613.712890;
    mMyntK.at<float>(1,0) = 0.0;        mMyntK.at<float>(1,1) = 713.578491; mMyntK.at<float>(1,2) = 386.504699;
    mMyntK.at<float>(2,0) = 0.0;        mMyntK.at<float>(2,1) = 0.0;        mMyntK.at<float>(2,2) = 1.0;
}

void Utils::ReadImgs(const std::string& pattern, std::vector<cv::Mat>& imgs)
{
    imgs.clear();
    vector<String> fn;
    glob(pattern, fn, false);
    size_t count = fn.size();

    for (size_t i = 0; i < count; i++)
    {
        imgs.push_back(imread(fn[i], 0));
    }
}

void Utils::ReadCameraPose(const std::string csv_path, std::vector<cv::Mat>& Tcws)
{
    Tcws.clear();

    ifstream file(csv_path);
    std::string line;
    int row = 0;
    double x,y,z, qx, qy, qz, qw;

    while(getline(file, line))
    {
        if(row==0)
        {
            row++;
            continue;       
        }

        stringstream ss(line);
        string str;
        
        int col = 0;
        double posx, posy, posz;
        double qx, qy, qz, qw;

        while (getline(ss, str, ','))
        {
            std::stringstream convertor(str);
            double value;
            convertor >> value;

            if(col==6)
                posx = value;
            else if (col==7)
                posy = value;
            else if (col==8)
                posz = value;
            else if (col==9)
                qx = value;
            else if (col==10)
                qy = value;
            else if (col==11)
                qz = value;
            else if (col==12)
                qw = value;

            col++;
        }

        cv::Mat Rwc = Quaternion2RotM(qx, qy, qz, qw);


        Mat Twc = (Mat_<float> (4,4) <<
            Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2), posx,
            Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2), posy,
            Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2), posz,
                             0,                0,                0,     1
        );

        Tcws.push_back(Twc.inv());
        row++;        
    }
}

cv::Mat Utils::Quaternion2RotM(float x, float y, float z, float w)
{
    float xx = x*x;
    float yy = y*y;
    float zz = z*z;

    float wx = w*x;
    float wy = w*y;
    float wz = w*z;

    float xy = x*y;
    float xz = x*z;
    float yz = y*z;

    Mat R = ( Mat_<float> ( 3,3 ) <<
                ( 1 - 2*yy - 2*zz ), ( 2*xy - 2*wz ), ( 2*xz + 2*wy ),
                ( 2*xy + 2*wz ), ( 1 - 2*xx - 2*zz ), ( 2*yz - 2*wx ),
                ( 2*xz - 2*wy ), ( 2*yz + 2*wx ), ( 1 - 2*xx - 2*yy ));

    return R;
}

