//
// Created by lu on 19-5-11.
//
#include "DTMunit.h"

/**
 * @brief 构建DTM的基本函数
 * @param initGood_matches
 * @param mvKeys1
 * @param mvKeys2
 * @param feature1
 * @param feature2
 * @return newGood_matches
 */
vector<DMatch> computeDTMunit(const vector<DMatch> &initGood_matches ,const vector<cv::KeyPoint> &mvKeys1,const vector<cv::KeyPoint> &mvKeys2, cv::Mat &feature1, cv::Mat &feature2 )
{
    Mat feature3 = feature1.clone();
    Mat feature4 = feature2.clone();
    ///delaunay one
    cout << "DT one:" << endl;
    vector<Vector2<float > > points1;
    for(const auto &p:initGood_matches)
    {
        points1.emplace_back(Vector2<float>(mvKeys1[p.queryIdx].pt.x , mvKeys1[p.queryIdx].pt.y ,p.imgIdx ));
    }

    Delaunay<float> triangulation1;
    const std::vector<Triangle<float> > triangles1 = triangulation1.triangulate(points1);  //逐点插入法
    triangulation1.computeEdgeMatrix();
    std::cout << "\t\t" <<triangles1.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges1 = triangulation1.getEdges();

    for(const auto &e : edges1)
    {
        line(feature1, Point(e.p1.x, e.p1.y), Point(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }

    ///delaunay two
    cout << "DT two:" << endl;
    vector<Vector2<float > > points2;
    for(const auto &p:initGood_matches)
    {
        points2.emplace_back(Vector2<float>(mvKeys2[p.trainIdx].pt.x , mvKeys2[p.trainIdx].pt.y ,p.imgIdx ));
    }

    Delaunay<float> triangulation2;
    const std::vector<Triangle<float> > triangles2 = triangulation2.triangulate(points2);  //逐点插入法
    triangulation2.computeEdgeMatrix();
    std::cout << "\t\t" <<triangles2.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges2 = triangulation2.getEdges();

    for(const auto &e : edges2)
    {
        line(feature2, Point(e.p1.x, e.p1.y), Point(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }

    /**************** 显示匹配结果与初始DT网络 ******************/
    cout << "\t匹配:" << endl;
    cout << "\t\tmatch:" << initGood_matches.size()<<endl;
    Mat beforeOpt;
    cv::drawMatches(feature1,mvKeys1,feature2,mvKeys2,initGood_matches,beforeOpt);
    imshow("before optimization",beforeOpt);
    waitKey(0);

/*******************  构建边矩阵，并计算相似度(范数)，进行DT网络的优化  *********************/
    cout << "\n计算DTM的相关信息：" << endl;
    Eigen::MatrixXd::Index maxRow,maxCol;
    Eigen::MatrixXd edgeMatrix = Eigen::MatrixXd::Zero(sizeofEdgeMatrix,sizeofEdgeMatrix);  //computeEdgeMatrix() 在此处也修改了 20,20 ，需要同步修改，后期改进此处
    edgeMatrix = triangulation1.getEdgeMatrix() - triangulation2.getEdgeMatrix();
    //    double value =0;
    //    value = edgeMatrix.norm();
    //    cout << "\tvalue: " << value <<  endl;      // 相似度

    edgeMatrix.cwiseAbs().colwise().sum().maxCoeff(&maxRow,&maxCol);    // 边矩阵.绝对值.列.和.最大值(行序号,列序号)

//    cout << "提取候选外点：\t"  << maxCol << endl;
//    cout << "显示sum:\n" << edgeMatrix.cwiseAbs().colwise().sum() << endl;
//    cout << "计算列和：\n" << edgeMatrix.cwiseAbs().colwise().sum()<< endl;
//    cout << "显示边矩阵之差：\n"<< edgeMatrix.cwiseAbs().col(maxCol).transpose() << endl;
//    cout << "二者之差：\n" << edgeMatrix.cwiseAbs().colwise().sum() - edgeMatrix.cwiseAbs().col(maxCol).transpose()<< endl;
//    cout << "候选外点：" << mvKeys2[good_matches[maxCol].trainIdx].pt << endl;

    // 通过DT网络的边矩阵之差的范数，删除列和较大的候选外点集
    vector<DMatch> newGood_matches(initGood_matches);
    cout << "\nold size:\t" << newGood_matches.size()<<endl;
    for(int i = sizeofEdgeMatrix;i != 0 ;i--)
    {
        if((edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) >= m_max_value )
        {
            cout << (edgeMatrix.cwiseAbs().colwise().sum())(0,i-1) << "\t,\t" << mvKeys1[newGood_matches[i-1].queryIdx].pt <<"\t,\t" << mvKeys2[newGood_matches[i-1].trainIdx].pt << endl;
            newGood_matches.erase(newGood_matches.begin()+i-1);
        }
    }
    cout << "new size:\t" << newGood_matches.size()<<endl;


    /************ 显示优化后的DT网络 ****************/
    ///delaunay three
    cout << "\tDT three:" << endl;
    std::vector<Vector2<float> > points3;
    for(const auto &g:newGood_matches)
    {
        points3.emplace_back(Vector2<float>(mvKeys1[g.queryIdx].pt.x , mvKeys1[g.queryIdx].pt.y ,g.imgIdx ));
    }
    Delaunay<float> triangulation3;
    const std::vector<Triangle<float> > triangles3 = triangulation3.triangulate(points3);  //逐点插入法
    triangulation3.computeEdgeMatrix();
    std::cout << "\t\t" << triangles3.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges3 = triangulation3.getEdges();
    for(const auto &e : edges3)
    {
        line(feature3, Point(e.p1.x, e.p1.y), Point(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }

    ///delaunay four
    cout << "\tDT four:" << endl;
    std::vector<Vector2<float> > points4;
    for(const auto &g:newGood_matches)
    {
        points4.emplace_back(Vector2<float>(mvKeys2[g.trainIdx].pt.x , mvKeys2[g.trainIdx].pt.y ,g.imgIdx ));
    }

    Delaunay<float> triangulation4;
    const std::vector<Triangle<float> > triangles4 = triangulation4.triangulate(points4);  //逐点插入法
    triangulation4.computeEdgeMatrix();
    std::cout << "\t\t" << triangles4.size() << " triangles generated"<<endl;
    const std::vector<Edge<float> > edges4 = triangulation4.getEdges();

    for(const auto &e : edges4)
    {
        line(feature4, Point(e.p1.x, e.p1.y), Point(e.p2.x, e.p2.y), Scalar(0, 0, 255), 1);
    }

    Mat afterOpt;
    cv::drawMatches(feature3,mvKeys1,feature4,mvKeys2,newGood_matches,afterOpt);
    imshow("after optimization",afterOpt);
    waitKey(0);
    /***********************************************/
    cout << "Finished in function!!!" << endl;
    return newGood_matches;
}

/**
 * @brief 获取剩余点集
 *
 * 输入
 * @param sizeofLevel             剩余点个数
 * @param good_matches
 * @param mvKeys1
 * @param mvKeys2
 * @param mDesc1
 * @param mDesc2
 *
 * 输出
 * @param mvKeys1_new
 * @param mvKeys2_new
 * @param mDes1_new
 * @param mDes2_new
 */
void updateKey(int sizeofLevel, const vector<DMatch> &good_matches, const vector<cv::KeyPoint> &mvKeys1, const vector<cv::KeyPoint> &mvKeys2, const cv::Mat &mDes1, const cv::Mat &mDes2,
               vector<cv::KeyPoint> &mvKeys1_new, vector<cv::KeyPoint> &mvKeys2_new, cv::Mat &mDes1_new, cv::Mat &mDes2_new)
{
    //   cv::Mat中没有删除某一列或者行的函数
    //   只能构造新的Mat，在删除某一列后，将后边的复制到新的Mat当中去
    //   新的解决方案是：将Mat转换为vector，使用back() pop()等操作处理后，再转换成Mat
    //   注意：由于删除的是列，而转换成vector后操作的是行，因此可以对Mat进行转置后，再进行转换操作，即Mat.t()
    //   在循环外边完成Mat到vector的转换工作，进行循环操作并退出后，再进行转换回来
    vector<int> order1,order2;
    cout << "Size of goodmatchs:  " << good_matches.size() << endl;
    // 更新特征点
    for(const auto &g:good_matches)
    {
        order1.emplace_back(g.queryIdx);
        order2.emplace_back(g.trainIdx);
    }
    sort(order1.begin(),order1.end());
    sort(order2.begin(),order2.end());

    // 更新描述子
    int dele_temp_1=0;
    int dele_temp_2=0;
    int dele_temp_count1=0;
    int dele_temp_count2=0;
    for (int i = 0; i < sizeofLevel; ++i)
    {
        if(i == *(order1.begin()+dele_temp_count1))
            dele_temp_count1++;
        else
        {
            mvKeys1_new.insert(mvKeys1_new.end(),mvKeys1.begin()+i,mvKeys1.begin()+i+1);
            mDes1.row(i).copyTo(mDes1_new.row(dele_temp_1));
            dele_temp_1++;
        }

        if(i == *(order2.begin()+dele_temp_count2))
            dele_temp_count2++;
        else
        {
            mvKeys2_new.insert(mvKeys2_new.begin()+dele_temp_2,mvKeys2.begin()+i,mvKeys2.begin()+i+1);
            mDes2.row(i).copyTo(mDes2_new.row(dele_temp_2));
            dele_temp_2++;
        }

    }
    cout << "Sizes of mvKeys1_new: \t" << mvKeys1_new.size() << endl;
    cout << "Sizes of mDes1_new:\t\t" << mDes1_new.size << endl;
    cout << "Sizes of mvKeys2_new: \t" << mvKeys2_new.size() << endl;
    cout << "Sizes of mDes2_new:\t\t" << mDes2_new.size << endl;

}


// sort()时，自定义的排序条件
// 用于对vector对象内的指定成员进行排序
inline bool cmp1(const DMatch first, const DMatch second)
{
    return first.trainIdx < second.trainIdx;
}

// unique()时，自定义的去重条件
// 用于对vector对象内的指定成员进行去重
inline bool cmp2(const DMatch first,const DMatch second)
{
    return first.trainIdx == second.trainIdx;
}
/**
 * @brief 使用BF匹配
 * @param mDes1
 * @param mDes2
 * @return
 */
vector<DMatch> BFmatchFunc(const cv::Mat &mDes1, const cv::Mat &mDes2, int threshold)
{
    cout << "\n显示第一次特征匹配的基本信息：" << endl;
    vector<DMatch> matches,good_matches;
    BFMatcher matcher (NORM_HAMMING);
    matcher.match(mDes1,mDes2,matches);

    //计算最大与最小距离
    double min_dist = 10000,max_dist = 0;

    for (int k = 0; k < mDes1.rows; k++)
    {
        double dist = matches[k].distance;
        if(dist < min_dist)
            min_dist = dist;
        if(dist > max_dist)
            max_dist = dist;
    }

    cout << "\tmin_dist:" << min_dist << endl;
    cout << "\tmax_dist:" << max_dist << endl;

    //筛选匹配
    int temp=0;
    for (int l = 0; l < mDes1.rows; l++)
    {
        if(matches[l].distance <= threshold )
        {
            matches[l].imgIdx=temp;
            good_matches.emplace_back(matches[l]);
            temp++;
        }
    }
    temp=0;

    sort(good_matches.begin(), good_matches.end(), cmp1);   //排序
    good_matches.erase(unique(good_matches.begin(),good_matches.end(),cmp2),good_matches.end());    //去重

    // 对新的排列重新赋值index
    for(int i =0 ;i < good_matches.size();i++)
    {
        good_matches[i].imgIdx = i;
    }

    return good_matches;
}


