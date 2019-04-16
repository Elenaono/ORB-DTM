#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <string>


using namespace std;
using namespace cv;

int threshold_value_one=0;
int pic_id = 1;

string file1 = "./data/desk1.png";
string file2 = "./data/desk2.png";
string file3 = "./data/4537.jpg";
string file4 = "./data/airplaneU2.bmp";
string file5 = "./data/mountain.bmp";
string file6 = "./data/test_lena.jpg";

string file11 = "./data/baboon.tiff";
string file12 = "./data/bridge.tiff";
string file13 = "./data/couple.tiff";
string file14 = "./data/crowd.tiff";
string file15 = "./data/lake.tiff";
string file16 = "./data/lax.tiff";
string file17 = "./data/lena.tiff";
string file18 = "./data/man.tiff";
string file19 = "./data/milkdrop.tiff";
//string file20 = "./data/peppers.tiff";
//string file21 = "./data/plane.tiff";
//string file22 = "./data/woman1.tiff";
//string file23 = "./data/woman2.tiff";

//string file11 = "./data/1.png";
//string file12 = "./data/2.png";
//string file13 = "./data/3.png";
//string file14 = "./data/4.png";
//string file15 = "./data/5.png";
//string file16 = "./data/6.png";
//string file17 = "./data/7.png";
//string file18 = "./data/8.png";
//string file19 = "./data/9.png";
string file20 = "./data/10.png";
string file21 = "./data/11.png";
string file22 = "./data/12.png";
string file23 = "./data/13.png";
string file24 = "./data/14.png";
string file25 = "./data/15.png";
string file26 = "./data/16.png";
string file27 = "./data/17.png";
string file28 = "./data/18.png";

int main()
{
/*****************************  读取图像文件   *****************************************************/
    string first_file ;
    switch(pic_id)
    {
        case 0:
            cerr << "no image!" << endl;
            break;
        case 1:
            first_file = file1;
            break;
        case 2:
            first_file = file2;
            break;
        case 3:
            first_file = file3;
            break;
        case 4:
            first_file = file4;
            break;
        case 5:
            first_file = file5;
            break;
        case 6:
            first_file = file6;
            break;
        case 11:
            first_file = file11;
            break;
        case 12:
            first_file = file12;
            break;
        case 13:
            first_file = file13;
            break;
        case 14:
            first_file = file14;
            break;
        case 15:
            first_file = file15;
            break;
        case 16:
            first_file = file16;
            break;
        case 17:
            first_file = file17;
            break;
        case 18:
            first_file = file18;
            break;
        case 19:
            first_file = file19;
            break;
        case 20:
            first_file = file20;
            break;
        case 21:
            first_file = file21;
            break;
        case 22:
            first_file = file22;
            break;
        case 23:
            first_file = file23;
            break;
        case 24:
            first_file = file24;
            break;
        case 25:
            first_file = file25;
            break;
        case 26:
            first_file = file26;
            break;
        case 27:
            first_file = file27;
            break;
        case 28:
            first_file = file28;
            break;
        default:
            cerr << "error!" << endl;
            break;
    }

    cv::Mat first_image = cv::imread(first_file, 0);    // load grayscale image 灰度图

//    imshow("show",first_image);
//    waitKey(0);
/*****************************  显示图像基本信息  *************************************************************/
//    cout<<"First\t"<< first_image.channels()<<endl;
//    cout<<"\t" <<first_image.rows<<"*"<<first_image.cols<<endl;   //column 列  row 行
    cout << "size:  "<<first_image.size<<endl;
/*****************************  计算图像对比度  *****************************************************/
    Mat mat_mean,mat_stddev;
    meanStdDev(first_image,mat_mean,mat_stddev);
    double m,s;
    m = mat_mean.at<double>(0,0);
    s = mat_stddev.at<double>(0,0);
    cout <<"灰度均值：   "<<m<<endl;
    cout <<"标准差：    "<<s<<endl;
/*****************************  提取Fast角点，并显示图像  *********************************************/
    float beta=0;
    if(s <40)
        beta = 1.5;
    else if(s>80)
        beta = 0.75;
    else
        beta = 1;

    // detect FAST keypoints using threshold=...
        threshold_value_one =( 1.05*s +2.2 )*beta;
        vector<cv::KeyPoint> keypoints;
        cv::FAST(first_image, keypoints, threshold_value_one, true);   //调用CV中的FAST算法，需自行加上旋转部分（即求解旋转角的函数），才组成ORB-FAST算法
        cout << "First keypoints: " << keypoints.size() <<"     threshold: "<< threshold_value_one<< endl;
//        cout << keypoints.size()<<endl;
//        cout << i << endl;

    // plot the keypoints
    cv::Mat image_show;
    cv::drawKeypoints(first_image, keypoints, image_show, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);//DRAW_OVER_OUTIMG     DRAW_RICH_KEYPOINTS
    cv::imshow("features", image_show);
//    cv::imwrite("feat1.png", image_show);
    cv::waitKey(0);
/******************************************************************************************/
    cout << "finsh" << endl;
    return 0;
}