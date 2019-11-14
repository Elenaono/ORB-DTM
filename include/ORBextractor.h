/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>

namespace ORB_SLAM2
{
    class ExtractorNode     //用于特征点剔除的八叉树节点
    {
    public:
        ExtractorNode():bNoMore(false){}    //构造函数
        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);    // 节点分配
        friend class ORBextractor;
    private:
        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    class ORBextractor
    {
    public:
        enum {HARRIS_SCORE=0, FAST_SCORE=1 };

        ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                     int iniThFAST, int minThFAST);     // ORB特征提取的构造函数
        ~ORBextractor()= default;
        void operator()( cv::InputArray image, cv::InputArray mask,
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::OutputArray descriptors);

        inline int GetLevels(){ return nlevels; }
        inline double GetScaleFactor(){ return scaleFactor; }
        inline std::vector<float> GetScaleFactors(){ return mvScaleFactor; }
        inline std::vector<float> GetInverseScaleFactors(){ return mvInvScaleFactor; }
        inline std::vector<float> GetScaleSigmaSquares(){ return mvLevelSigma2; }
        inline std::vector<float> GetInverseScaleSigmaSquares(){ return mvInvLevelSigma2; }
        std::vector<cv::Mat> mvImagePyramid;    //图像金字塔 存放各层的图片

    protected:
        void ComputePyramid(const cv::Mat& image);     //计算高斯金字塔
        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);//计算关键点并用四叉树(八叉树)进行存储
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                                    const int &maxX, const int &minY, const int &maxY,
                                                    const int &nFeatures, const int &level);
        void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
        std::vector<cv::Point> pattern;     //存储关键点附近patch的点对

        int nfeatures;      //提取特征点的最大数量
        double scaleFactor; //每层之间的缩放比例
        int nlevels;        //高斯金字塔的层数
        int iniThFAST;      //initial threshold for FAST extract
        int minThFAST;      //minimum threshold for FAST extract

        std::vector<int> mnFeaturesPerLevel;    //每层的特征点数
        std::vector<int> umax;  //Patch元的最大坐标
        std::vector<float> mvScaleFactor;       //每层的相对于原始图像的缩放比例
        std::vector<float> mvInvScaleFactor;    //每层的相对于原始图像的缩放比例的倒数
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;

    public:     //my functions
        std::vector<std::vector<cv::KeyPoint> > mvvKeypoints;
        std::vector<cv::Mat> inline GetImagePyramid() { return mvImagePyramid; }
        std::vector<int> inline GetmnFeaturesPerLevel() { return mnFeaturesPerLevel; }
        std::vector<std::vector<cv::KeyPoint> > inline GetmvvKeypoints() { return mvvKeypoints; }
    };

} //namespace ORB_SLAM

#endif

