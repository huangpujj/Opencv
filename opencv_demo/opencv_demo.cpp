﻿#include<opencv2/objdetect/objdetect.hpp>  
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>  
#include<opencv2/video/video.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;

void drawCornerOnImage(Mat& image, const Mat& binary)
{
    Mat_<uchar>::const_iterator it = binary.begin<uchar>();
    Mat_<uchar>::const_iterator itd = binary.end<uchar>();
    for (int i = 0; it != itd; it++, i++)
    {
        if (*it)
            circle(image, Point(i % image.cols, i / image.cols), 3, Scalar(0, 255, 0), 1);
    }
}

int main()
{
    Mat image = imread("C:\\Users\\huang\\Desktop\\VS_repos\\videos\\sample.png");
    Mat gray;
    cvtColor(image, gray, 7);

    Mat cornerStrength;
    cornerHarris(gray, cornerStrength, 3, 3, 0.01);

    double maxStrength;
    double minStrength;
    // 找到图像中的最大、最小值
    minMaxLoc(cornerStrength, &minStrength, &maxStrength);

    Mat dilated;
    Mat locaMax;
    // 膨胀图像，最找出图像中全部的局部最大值点
    dilate(cornerStrength, dilated, Mat());
    // compare是一个逻辑比较函数，返回两幅图像中对应点相同的二值图像
    compare(cornerStrength, dilated, locaMax, CMP_EQ);

    Mat cornerMap;
    double qualityLevel = 0.01;
    double th = qualityLevel * maxStrength; // 阈值计算
    threshold(cornerStrength, cornerMap, th, 255, THRESH_BINARY);
    cornerMap.convertTo(cornerMap, CV_8U);
    // 逐点的位运算
    bitwise_and(cornerMap, locaMax, cornerMap);

    drawCornerOnImage(image, cornerMap);
    namedWindow("result");
    imshow("result", image);
    waitKey();

    return 0;
}
