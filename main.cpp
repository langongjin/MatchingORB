#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/legacy/legacy.hpp>

using namespace cv;
using namespace std;
int main()
{
    Mat img_1 = imread("/Users/lan/Desktop/TarReg/ORB/ORBtest/church01.jpg");
    Mat img_2 = imread("/Users/lan/Desktop/TarReg/ORB/ORBtest/church02.jpg");
    if (!img_1.data || !img_2.data)
    {
        cout << "error reading images " << endl;
        return -1;
    }

    ORB orb;
    vector<KeyPoint> keyPoints_1, keyPoints_2;
    Mat descriptors_1, descriptors_2;

    orb(img_1, Mat(), keyPoints_1, descriptors_1);
    orb(img_2, Mat(), keyPoints_2, descriptors_2);

    BruteForceMatcher<Hamming> matcher;
    vector<DMatch> matches;
    matcher.match(descriptors_1, descriptors_2, matches);

    double max_dist = 0; double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );
    //-- Draw only "good" matches (i.e. whose distance is less than 0.6*max_dist )
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        if( matches[i].distance < 0.5*max_dist )
        {
            good_matches.push_back( matches[i]);
        }
    }

    Mat img_matches;
    drawMatches(img_1, keyPoints_1, img_2, keyPoints_2,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    imshow( "MatchingORB", img_matches);
    cvWaitKey();
    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *@function ORB_Detect.cpp
 *@brief 使用ORB特征对目标进行检测和匹配
 *@author ltc
 *@date 14:45 Thursday，December 3rd，2015
*/


//using namespace std;
//using namespace cv;
//
//vector<DMatch> ransac(vector<KeyPoint> queryKeyPoint,vector<KeyPoint> trainKeyPoint,vector<DMatch> matches);
//
//int main(int argc,char* argv[])
//{
//    Mat queryImage,trainImage;
//
//    queryImage=imread("4.jpg",IMREAD_COLOR);
//    trainImage=imread("3.jpg",IMREAD_COLOR);
//
//    ORB orb;
//
//    vector<KeyPoint> queryKeyPoint,trainKeyPoint;
//    Mat queryDescriptor,trainDescriptor;
//
//    orb(queryImage,Mat(),queryKeyPoint,queryDescriptor);
//    orb(trainImage,Mat(),trainKeyPoint,trainDescriptor);
//
//    drawKeypoints(queryImage,queryKeyPoint,queryImage);
//    drawKeypoints(trainImage,trainKeyPoint,trainImage);
//
//    cout<<queryKeyPoint.size()<<endl;
//    cout<<trainKeyPoint.size()<<endl;
//
//    imshow("query",queryImage);
//    imshow("train",trainImage);
//
//    //NORM_L1, NORM_L2, NORM_HAMMING, NORM_HAMMING2
//    BFMatcher matcher(NORM_L2);
//
//    vector<DMatch> matches;
//    matcher.match(queryDescriptor,trainDescriptor,matches);
//
//    vector<DMatch> ransac_matches;
//    ransac_matches=ransac(queryKeyPoint,trainKeyPoint,matches);
//
//    Mat image_match;
////  drawMatches(queryImage,queryKeyPoint,trainImage,trainKeyPoint,matches,image_match);
//    drawMatches(queryImage,queryKeyPoint,trainImage,trainKeyPoint,ransac_matches,image_match);
//
//    imshow("image_match",image_match);
//    waitKey(0);
//
//    return 0;
//}
//vector<DMatch> ransac(vector<KeyPoint> queryKeyPoint,vector<KeyPoint> trainKeyPoint,vector<DMatch> matches)
//{
//    vector<DMatch> ransac_matches;
//    vector<Point2f> queryPoint(matches.size()),trainPoint(matches.size());
//    for(int i=0;i<matches.size();i++)
//    {
//        queryPoint[i]=queryKeyPoint[matches[i].queryIdx].pt;
//        trainPoint[i]=trainKeyPoint[matches[i].trainIdx].pt;
//    }
//    vector<unsigned char> inlierMask(matches.size());
//    Mat H=findHomography(queryPoint,trainPoint,RANSAC,3,inlierMask);
//
//    for(size_t i=0;i<inlierMask.size();i++)
//    {
//        if(inlierMask[i])
//        {
//            ransac_matches.push_back(matches[i]);
//        }
//    }
//    return ransac_matches;
//}