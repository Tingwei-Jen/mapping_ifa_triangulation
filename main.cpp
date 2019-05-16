#include "localmapping.h"
#include "utils.h"
#include <iostream>
using namespace std;
using namespace cv;

Point Vertex(-1,-1);
void onMouse(int Event,int x,int y,int flags,void* param);

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
    vector<Point2f> vertexs10;
    vertexs10.push_back(Point2f(873, 542));
    vertexs10.push_back(Point2f(1026, 555));
    vertexs10.push_back(Point2f(1039, 660));
    vertexs10.push_back(Point2f(878, 646));
    

    //vertex of traffic sign0 in img5
    vector<Point2f> vertexs05;
    vertexs05.push_back(Point2f(858,517));
    vertexs05.push_back(Point2f(1014,534));
    vertexs05.push_back(Point2f(1025,648));
    vertexs05.push_back(Point2f(856,630));

    //vertex of traffic sign1 in img5
    vector<Point2f> vertexs15;
    vertexs15.push_back(Point2f(507,298));
    vertexs15.push_back(Point2f(728,331));
    vertexs15.push_back(Point2f(705,498));
    vertexs15.push_back(Point2f(486,485));
    

    //vertex of traffic sign0 in img8
    vector<Point2f> vertexs08;
    vertexs08.push_back(Point2f(528,233));
    vertexs08.push_back(Point2f(769,269));
    vertexs08.push_back(Point2f(744,457));
    vertexs08.push_back(Point2f(505,437));

    //vertex of traffic sign0 in img10
    vector<Point2f> vertexs010;
    vertexs010.push_back(Point2f(979,458));
    vertexs010.push_back(Point2f(1176,483));
    vertexs010.push_back(Point2f(1191,622));
    vertexs010.push_back(Point2f(972,594));

    //vertex of traffic sign1 in img10
    vector<Point2f> vertexs110;
    vertexs110.push_back(Point2f(543,187));
    vertexs110.push_back(Point2f(810,232));
    vertexs110.push_back(Point2f(779,435));
    vertexs110.push_back(Point2f(517,413));

    Detection ref;
    ref.img = imgs[0];
    ref.vVertexess.push_back(vertexs00);
    ref.vVertexess.push_back(vertexs10);

    Detection ref2;
    ref2.img = imgs[5];
    ref2.vVertexess.push_back(vertexs05);
    ref2.vVertexess.push_back(vertexs15);

    Detection ref3;
    ref3.img = imgs[8];
    ref3.vVertexess.push_back(vertexs08);

    Detection ref4;
    ref4.img = imgs[10];
    ref4.vVertexess.push_back(vertexs010);
    ref4.vVertexess.push_back(vertexs110);


    localmapping.UpdateReference(ref);
    localmapping.AddNewImg(imgs[0], Tcws[0]);
    localmapping.Run();
    localmapping.AddNewImg(imgs[1], Tcws[1]);
    localmapping.Run();
    localmapping.AddNewImg(imgs[2], Tcws[2]);
    localmapping.Run();
    localmapping.AddNewImg(imgs[3], Tcws[3]);
    localmapping.Run();
    localmapping.AddNewImg(imgs[4], Tcws[4]);
    localmapping.Run();
    localmapping.UpdateReference(ref2);
    localmapping.AddNewImg(imgs[5], Tcws[5]);
    localmapping.Run();
    localmapping.AddNewImg(imgs[6], Tcws[6]);
    localmapping.Run();
    localmapping.AddNewImg(imgs[7], Tcws[7]);
    localmapping.Run();
    localmapping.UpdateReference(ref3);
    localmapping.AddNewImg(imgs[8], Tcws[8]);
    localmapping.Run();
    localmapping.AddNewImg(imgs[9], Tcws[9]);
    localmapping.Run();
    localmapping.UpdateReference(ref4);
    localmapping.AddNewImg(imgs[10], Tcws[10]);
    localmapping.Run();

    vector<Sign*> signs = localmapping.GepMap()->GetAllSigns();
    cout<<signs[0]->GetVertexes3D()[0]<<endl;










    // Mat img = imgs[10];

    // namedWindow("image",0);
    // setMouseCallback("image",onMouse,NULL);

    // while(true)
    // {
    //     if(Vertex.x==-1)
    //     {
    //         imshow("image", img);
    //     }

    //     if(cvWaitKey(33)==27){
    //         break;
    //     }
    // }


    //draw
    // Mat color0, color1; 
    // cv::cvtColor(img0, color0, cv::COLOR_GRAY2BGR);   
    // cv::cvtColor(img1, color1, cv::COLOR_GRAY2BGR);   
    // imshow("color0", color0);
    // imshow("color1", color1);
    // waitKey(0);

    return 0;
}

void onMouse(int Event,int x,int y,int flags,void* param)
{    
    if(Event==CV_EVENT_LBUTTONDOWN)
    {
        Vertex.x = x;
        Vertex.y = y;
        cout<<"x: "<<x<<"   y: "<<y<<endl;
    }
}
