#ifndef CURBDETECTION_CURBDETECT_H
#define CURBDETECTION_CURBDETECT_H

#include <iostream>
#include <vector>
#include <chrono>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include "define.h"

using namespace std;
namespace CurbDetect{


//路沿
class Line{
public:

    cv::Point2f pt1, pt2, pt_low, pt_mid, new_pt1, new_pt2, new_pt_mid;         //变长点，便于灵活绘制不同长度的直线
    //pt_bottom 直线与图像下方交点  pt_top 和pt_bottom对应该条直线的定长数据， 此时pt_track 为跟踪参照点(定长中点）
    cv::Point2f pt_bottom, pt_top, pt_track;
    double theta, angle, delta_x, delta_y,  new_delta_x, new_delta_y, neighbor_delta_x;
    double k,b;     // 直线方程为：y = kx+b 利用中点和角度求解

    Line(){
        neighbor_delta_x = 0;
    };
    ~Line(){};
    void init()
    {
        cv::Point2f tmp(0,0);
        pt1 = tmp;
        pt_track=pt_top=pt_bottom=pt2=pt_low=pt_mid=new_pt1=new_pt2=new_pt_mid=pt1;
        k = 0.01;
        theta=angle=delta_x=delta_y=new_delta_x=new_delta_y=neighbor_delta_x=b=k;
    }

    void stretch_line(double scale,cv::Point2f new_low_pt);



private:


};

//目标路沿线
class Target{
public:

    Line goal_line_curr;
    Line goal_line_pre;
    int state = 0;      // 0-lost  1-drop 2-track
    int last_state = 0;
    int drop_cnt = 0;
    int find_cnt = 0;


    Target(){};
    Target(Line l)
    {
        this->goal_line_curr = l;
    }
    void init()
    {
        int state = 0;
        int last_state = 0;
        int drop_cnt = 0;
        int find_cnt = 0;
        this->goal_line_curr.init();
        this->goal_line_pre = this->goal_line_curr;

    }
    ~Target(){};

private:


};

    //路沿识别器
class CurbDetector{
public:
    vector<Line> lines_;
    Target target_curr, target_pre, target_goal, target_init;
    vector<Target> line_targets;        //一定历史帧数的goal_line


    cv::Mat src;
    cv::Mat gray_img;
    cv::Mat gauss_img;
    cv::Mat hough_img;
    cv::Mat canny_img;
    cv::Mat thre_img;
    cv::Mat erode_img, dilate_img;
    cv::Mat roi,roi_gray;

    int low_thre, high_thre, low_canny, high_canny, hough_thre, minLineLenght,maxLineGap;


    //二维算子
    cv::Mat kernel11,kernel33,kernel55,kernel77,kernel99;
    //一维算子
    cv::Mat kernel1;


    CurbDetector(cv::Mat src){
        this->src = src;
    };
    CurbDetector(){
        target_goal.init();
        line_targets.clear();
        lines_.clear();
        low_thre = 180;
        high_thre = 238;
        low_canny = 0;
        high_canny = 255;


        maxLineGap = 100;
        hough_thre = 100;
        minLineLenght = 100;

#ifdef GAUSS
        low_thre =180;
#else
      low_thre =185;
#endif

#ifndef THRE
    low_canny = 75;
    high_canny = 255;
    hough_thre = 20;
    minLineLenght = 30;
#endif


        //滑动条
        cv::namedWindow("SetParam",0);
        cv::createTrackbar("low_thre","SetParam",&low_thre,255);
        cv::createTrackbar("high_thre","SetParam",&high_thre,255);
        cv::createTrackbar("low_canny","SetParam",&low_canny,255);
        cv::createTrackbar("high_canny","SetParam",&high_canny,255);
        cv::createTrackbar("hough_thre","SetParam",&hough_thre,255);
        cv::createTrackbar("minLineLenght","SetParam",&minLineLenght,255);
        cv::createTrackbar("maxLineGap","SetParam",&maxLineGap,255);
    };
    ~CurbDetector(){};

    void curb_detect_steps(cv::Mat src);
    void predeal_img();         //预处理
    void update_target();
    void show_img();
    cv::Mat get_rect_roi_img(cv::Mat src,cv::Point start_point,double x_scale,double y_scale,cv::Rect & replace_roi);
    void draw_legal_lines(vector<Line> lines,cv::Mat & src);            //绘制合法直线
    void draw_target_line(Target t,cv::Mat & src);                   //绘制目标
    vector<cv::Point2f> draw_HoughLines(cv::Mat & src, vector<cv::Vec2f> lines);
    vector<cv::Point2f> draw_HoughLinesP(cv::Mat & src, vector<cv::Vec4i> lines);
    void pop_target(int index);
    void insert_target(Target t);
private:

};











}


#endif //CURBDETECTION_CURBDETECT_H
