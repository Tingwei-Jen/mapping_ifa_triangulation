#include "localmapping.h"
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
    Map* map = new Map();
    LocalMapping localmapping(K, map);

    //vertex of traffic sign0 in img0
    vector<Point2f> vertexs00;
    vertexs00.push_back(Point2f(545, 356));
    vertexs00.push_back(Point2f(747, 376));
    vertexs00.push_back(Point2f(733, 530));
    vertexs00.push_back(Point2f(532, 520));

    //vertex of traffic sign1 in img0
    vector<Point2f> vertexs01;
    vertexs01.push_back(Point2f(873, 542));
    vertexs01.push_back(Point2f(1026, 555));
    vertexs01.push_back(Point2f(1039, 660));
    vertexs01.push_back(Point2f(878, 646));
    
    Detection ref;
    ref.img = imgs[0];
    ref.vVertexess.push_back(vertexs00);
    ref.vVertexess.push_back(vertexs01);

    localmapping.UpdateReference(ref);
    localmapping.AddNewImg(imgs[0], Tcws[0]);
    localmapping.Run();
    localmapping.AddNewImg(imgs[1], Tcws[1]);
    localmapping.Run();
    // localmapping.AddNewImg(imgs[2], Tcws[2]);
    // localmapping.Run();


    //draw
    // Mat color0, color1; 
    // cv::cvtColor(img0, color0, cv::COLOR_GRAY2BGR);   
    // cv::cvtColor(img1, color1, cv::COLOR_GRAY2BGR);   
    // imshow("color0", color0);
    // imshow("color1", color1);
    // waitKey(0);

    return 0;
}
