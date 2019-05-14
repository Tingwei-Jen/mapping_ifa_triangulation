#include "utils.h"
#include <iostream>
using namespace std;
using namespace cv;

int main()
{

    Utils utils;
    string pattern = "/home/tom/VIO_dataset/vins_mynteye_experiment_data/IFA_CVS_exp/imgs/*.png";
    string poseCSV = "/home/tom/VIO_dataset/vins_mynteye_experiment_data/IFA_CVS_exp/IFA_CVS_exp1-vins_estimator-camera_pose.csv";
    vector<Mat> imgs;
    utils.ReadImgs(pattern, imgs);
    vector<Mat> Tcws;
    utils.ReadCameraPose(poseCSV, Tcws);
    
    Mat K = utils.GetMyntK();

    //draw
    // Mat color0, color1; 
    // cv::cvtColor(img0, color0, cv::COLOR_GRAY2BGR);   
    // cv::cvtColor(img1, color1, cv::COLOR_GRAY2BGR);   
    // imshow("color0", color0);
    // imshow("color1", color1);
    // waitKey(0);

    return 0;
}
