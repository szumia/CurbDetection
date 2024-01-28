#include "curbDetect.h"


namespace CurbDetect{

    //x小排前
    bool comLineX(Line l1,Line l2)
    {
        return l1.pt_bottom.x < l2.pt_bottom.x;
    }


    /*
     *new_low_pt 新的最低点
     */
    void Line::stretch_line(double scale,cv::Point2f new_low_pt)
    {
        double scale_x,scale_y;
        scale_x  = abs(this->new_delta_x  * scale);
        scale_y = abs(this->new_delta_y * scale);
        cv::Point2f delta_pt = cv::Point2f(scale_x,scale_y);
        this->new_pt1 = new_low_pt - delta_pt;
        this->new_pt2 = new_low_pt;
        this->new_pt_mid = (new_pt1 + new_pt2)/2;
        this->new_delta_x = this->new_pt2.x - this->new_pt1.x;
        this->new_delta_y = this->new_pt2.y - this->new_pt1.y;


    }


    void CurbDetector::curb_detect_steps(cv::Mat src)
    {
        this->src = src.clone();
        this->lines_.clear();
        if(!this->src.empty())
        {
//            cout<<"video frame width: "<< this->src.cols<<"  height: "<< this->src.rows<<endl;
            this->kernel11 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,1));
            this->kernel33 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
            this->kernel55 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
            this->kernel77 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));
            this->kernel99 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9,9));
            this->predeal_img();
            this->update_target();
            this->show_img();
            if(target_goal.state == 0)
            {
                cout<<"lost ";
            } else if(target_goal.state == 1)
            {
                cout<<"drop ";
            } else if(target_goal.state == 2)
            {
                cout<<"track ";
            }
            cout<<" drop_cnt: "<<target_goal.drop_cnt<<"  find_cnt: "<<target_goal.find_cnt<<endl<<endl;
        }else
        {;}

    }

    //预处理
    //远处本身就模糊，不做处理
    //近处做高斯模糊处理
    void CurbDetector::predeal_img()
    {

        cv::GaussianBlur(this->src, this->gauss_img, this->kernel55.size(),0,0);
#ifdef GAUSS
                cv::cvtColor(this->gauss_img, this->gray_img,cv::COLOR_BGR2GRAY);                         //全图使用高斯模糊
#else
        cv::cvtColor(this->src, this->gray_img,cv::COLOR_BGR2GRAY);                                  //全图不使用高斯模糊
#endif


#ifdef THRE
        cv::threshold(this->gray_img, this->thre_img,185,255,cv::THRESH_BINARY);

        //右下 1/2 区域进行均值/高斯滤波,再进行二值化
        cv::Point start_p(this->src.cols/2, this->src.rows/2);
        cv::Rect replace_roi;
        cv::Mat roi_rd = this->get_rect_roi_img(this->gray_img,start_p,0.5,0.5,replace_roi);
        cv::Mat blur_roi_rd,thre_roi_rd;
                cv::blur(roi_rd,blur_roi_rd,kernel55.size());   //均值滤波
//        cv::GaussianBlur(roi_rd,blur_roi_rd,this->kernel55.size(),0,0);         //高斯滤波
//        cv::threshold(blur_roi_rd,thre_roi_rd,170,255,cv::THRESH_BINARY);
        cv::namedWindow("BlurRoiRD",cv::WINDOW_NORMAL);
        cv::imshow("BlurRoiRD",blur_roi_rd);
//        cv::namedWindow("ThreRoiRD",cv::WINDOW_NORMAL);
//        cv::imshow("ThreRoiRD",thre_roi_rd);


//        cout<<"threshold low: "<<low_thre<<"  high: "<<high_thre<<endl;

        //原thre图右下1/2区域替换
        blur_roi_rd.copyTo(this->thre_img(replace_roi));
//        thre_roi_rd.copyTo(this->thre_img(replace_roi));
        //再次二值化
        cv::threshold(this->thre_img, this->thre_img,170,255,cv::THRESH_BINARY);
#else

#endif

        //提取路沿roi (不规则四边形）
        cv::Mat mask(this->src.rows, this->src.cols,CV_8UC1,cv::Scalar (0));    //全黑mask
        cv::Point lt,rt,rd,ld;                //左上顺时针
        lt = cv::Point (280,70);
        rt =cv::Point (320,70);
        rd = cv::Point (530,480);
        ld = cv::Point (340,480);
        vector<cv::Point > roi_pts;
        roi_pts.push_back(lt);
        roi_pts.push_back(rt);
        roi_pts.push_back(rd);
        roi_pts.push_back(ld);
        cv::fillPoly(mask,roi_pts,cv::Scalar (255));                     //全白填充roi
//        cv::namedWindow("mask",cv::WINDOW_NORMAL);
//        cv::imshow("mask",mask);

#ifdef THRE
        this->thre_img = this->thre_img & mask;
//        cv::GaussianBlur(this->thre_img, this->thre_img, this->kernel33.size(),0,0);
//        cv::medianBlur(this->thre_img, this->thre_img, 3);//中值滤波滤除椒盐噪声

//        cv::erode(this->thre_img, this->thre_img,kernel33);             //腐蚀thre_img
//        cv::dilate(this->thre_img, this->thre_img,kernel11);            //膨胀thre_img
//        this->thre_img = ~this->thre_img;

        std::vector<cv::Mat> channels;
        cv::split(this->src, channels);
        cv::Scalar bl = cv::mean(channels[0]);
        low_canny = (int )(0.4 * bl[0]);
        high_canny = (int )(1.2 * bl[0]);


        cv::Canny(this->thre_img, this->canny_img,low_canny,high_canny); //thre canny
//        cout<<"canny low: "<<low_canny<<"  high: "<<high_canny<<endl;
//        cv::dilate(this->canny_img, this->canny_img,kernel11);            //先膨胀thre_img
//        cv::erode(this->canny_img, this->canny_img,kernel11);             //再腐蚀thre_img
#else
        this->gray_img = this->gray_img & mask;
        cv::Canny(this->gray_img, this->canny_img,low_canny,high_canny); //thre canny
#endif




//        this->roi_gray = this->gray_img & mask;
//        cv::GaussianBlur(this->roi_gray, this->roi_gray, this->kernel33.size(),0,0);
//        cv::Canny(this->gray_img, this->canny_img,low_canny,high_canny);              //全图canny
//        cv::Canny(this->roi_gray, this->canny_img,low_canny,high_canny);         //roi canny


        //霍夫直线检测
        vector<cv::Vec4i> lines;
        vector<cv::Point2f>lines_pts;
        cv::HoughLinesP(this->canny_img,lines,1,CV_PI/180, this->hough_thre, this->minLineLenght, this->maxLineGap);
//        cout<<"total lines size: "<<lines.size()<<endl;

        cv::Point2f pt1,pt2,pt_low,pt_mid;
        float x1,y1,x2,y2,x0,y0;
        double theta,angle;
        for(size_t i = 0; i <lines.size(); i++)
        {
            Line line_;
            x1 = lines[i][0];
            y1 = lines[i][1];
            x2 = lines[i][2];
            y2 = lines[i][3];
            x0 =(x1 + x2) / 2;
            y0 =(y1 + y2) / 2;
            pt1=cv::Point2f (x1,y1);
            pt2=cv::Point2f (x2,y2);
            pt_mid=cv::Point2f (x0,y0);
            line_.pt_mid = pt_mid;
            line_.new_pt_mid =pt_mid;

            //靠近图像下方的point为pt2
            if(pt1.y >= y0)
            {
                pt_low =pt1;
               line_.pt2 = pt1;
               line_.pt1 = pt2;
            } else{
                pt_low = pt2;
                line_.pt2 = pt2;
                line_.pt1 = pt1;
            }
            line_.new_pt1 = line_.pt1;
            line_.new_pt2 = line_.pt2;

            angle = atan((pt_low.y - y0)/(pt_low.x -x0));
            //求解k,b
            line_.k = tan(angle);
            line_.b = line_.pt_mid.y - line_.k*line_.pt_mid.x;
            //求解直线和图像bottom的交点
            line_.pt_bottom.y = this->src.rows;
            line_.pt_bottom.x = (line_.pt_bottom.y - line_.b)/line_.k;
            float x_tmp, y_tmp, x_sub, y_sub;
            x_sub = LINE_STATIC_LENGTH * cos(angle);
            y_sub =LINE_STATIC_LENGTH * sin(angle);
            line_.pt_top.x = line_.pt_bottom.x - x_sub;
            line_.pt_top.y =line_.pt_bottom.y - y_sub;
            line_.pt_track = (line_.pt_top + line_.pt_bottom)/2;


            line_.angle =angle * 180 / CV_PI;
            line_.delta_x = line_.pt2.x - line_.pt1.x;
            line_.delta_y = line_.pt2.y - line_.pt1.y;
            line_.new_delta_x = line_.delta_x;
            line_.new_delta_y = line_.delta_y;
            line_.stretch_line(0.6,line_.pt2);


            //筛选
            if(line_.angle > LINE_ANGLE_UP)
                continue;
            if(line_.angle < LINE_ANGLE_LOW)
                continue;

            lines_.push_back(line_);
        }


        //再次匹配直线（去掉最右侧1/2直线）
        if(lines_.size() >= 2)
        {
            sort(lines_.begin(),lines_.end(), comLineX);
            int erase_sum = lines_.size()/2;
            //从后往前删
            for(size_t i = 0; i < erase_sum; i++)
            {
                lines_.pop_back();
            }
        }


        if(lines_.size() >= 2)
        {
            //从左至右，计算相邻直线间的neighbor_delta_x
            for(size_t i = 0 ;i < lines_.size() - 1; i++)
            {
                lines_[i].neighbor_delta_x = lines_[i+1].pt_bottom.x - lines_[i].pt_bottom.x;
            }
            //从右往左，删除相邻距离过大的
            for(size_t i = 0;i < lines_.size()-1;i++)
            {
                if(lines_[lines_.size() - 2].neighbor_delta_x > NEIGHBOR_DELTA_X_UP)
                    lines_.pop_back();
            }
        }

//        Target t;
//        //一般偏左边那条是路沿线
//        if(lines_.size() > 0)
//        {
//            t.goal_line_curr = lines_[0];
//            this->target_curr = t;
//            this->target_pre = this->target_curr;
////            insert_target(t);
//        }




    }

    void CurbDetector::update_target()
    {
        Target t;
        //一般偏左边那条是路沿线
        if(lines_.size() > 0)
        {

            t.goal_line_curr = lines_[0];
            cout<<"find line! line mid y is "<<t.goal_line_curr.pt_mid.y<<endl;
//            this->target_curr = t;
//            this->target_pre = this->target_curr;
////            insert_target(t);
        } else
        {
            t.init();
            cout<<"not find line! line mid y is "<<t.goal_line_curr.pt_mid.y<<endl;
        }


        //lost
        if(target_goal.state == 0 && t.goal_line_curr.pt_mid.y != 0)
        {
            //lost后，初始为最新一帧
           target_goal.drop_cnt = 0;
           target_goal.find_cnt += 1;
           this->insert_target(t);
           target_goal.state = 2;

        }
        else if(target_goal.state != 0)
        {
            bool is_drop = true;
            double min_dis = std::numeric_limits<double>::max();
            double tmp_dis = 0;
            int is_left = 0;       //新选择的线方向 0：默认  -1：左边  1：右边
            Line min_dis_l;
            Target min_dis_t;
            min_dis_l.init();
            min_dis_t.init();
            //找track_point距离最短的
            for(size_t i=0; i<lines_.size();i++)
            {
                tmp_dis = cv::norm(target_goal.goal_line_curr.pt_mid -lines_[i].pt_mid);
                if(tmp_dis < min_dis)
                {
                    min_dis = tmp_dis;
                    min_dis_l = lines_[i];
                }
            }
            min_dis_t.goal_line_curr = min_dis_l;
            if(min_dis_l.pt_bottom.x == 0)
            {
                is_left = 0;
            } else if((min_dis_l.pt_bottom.x - target_goal.goal_line_curr.pt_bottom.x)>0)
            {
                is_left =1;
            } else
            {
                is_left = -1;
            }
            cout<<"min dis: "<<min_dis<<endl;



            if(min_dis_t.goal_line_curr.pt_mid.y == 0 )
            {
                is_drop = true;
            }
            else if(min_dis > MAX_DROP_DELTA_X && is_left == 1)                             //距离过大 且 新选择的线要在右边，判为drop
                is_drop = true;
            else
                is_drop = false;

            if(is_drop == false)
            {
                target_goal.state = 2;
                target_goal.drop_cnt = 0;
                target_goal.find_cnt += 1;
                this->insert_target(min_dis_t);
            } else
            {
                target_goal.state = 1;
                target_goal.drop_cnt += 1;
                target_goal.find_cnt += 0;

                //超过最大掉帧数，init
                if(target_goal.drop_cnt > MAX_DROP_CNT)
                {
                    target_goal.init();
                }
            }
        }


        /*set init target*/
        if(target_goal.state == 0)
        {
            if(t.goal_line_curr.pt_mid.y != 0)
            {
                target_goal.goal_line_curr=t.goal_line_curr;
            } else
            {;}
        }
        else
        {
            target_pre = line_targets.back();
            if(target_pre.goal_line_curr.pt_mid.y != 0)
            {
                target_goal.goal_line_curr = target_pre.goal_line_curr;
            } else
            {;}
        }
        cout<<target_goal.state<<endl;

    }


    /*
     * 起点向右截取scale范围
     */
    cv::Mat CurbDetector::get_rect_roi_img(cv::Mat src,cv::Point start_point,double x_scale,double y_scale, cv::Rect & replace_roi)
    {
        int width,height;
        width = src.cols;
        height = src.rows;
        int rect_rows, rect_cols;
        rect_cols = width * x_scale;
        rect_rows = height * y_scale;
        cv::Rect roi_rect(start_point.x, start_point.y, rect_cols, rect_rows);
        replace_roi = roi_rect;
        cv::Mat result = cv::Mat(src,roi_rect);
        return result;
    }

    //绘制目标
    void CurbDetector::draw_target_line(Target t,cv::Mat & src)
    {
        //绘制变长曲线
//            cv::line(src,lines[i].new_pt1,lines[i].new_pt2,cv::Scalar (255,0,0),2,cv::LINE_AA);         //LINE_AA 抗锯齿
        //绘制定长曲线
        cv::line(src,t.goal_line_curr.pt_top,t.goal_line_curr.pt_bottom,cv::Scalar (0,0,255),2,cv::LINE_AA);         //LINE_AA 抗锯齿
        //绘制跟踪点
        cv::circle(src,t.goal_line_curr.pt_track,4.5,cv::Scalar (0,255,0),-1);
    }



    void CurbDetector::draw_legal_lines(vector<Line> lines,cv::Mat & src)
    {
        for(size_t i = 0; i < lines.size();i++) {
            //绘制变长曲线
//            cv::line(src,lines[i].new_pt1,lines[i].new_pt2,cv::Scalar (255,0,0),2,cv::LINE_AA);         //LINE_AA 抗锯齿
            //绘制定长曲线
            cv::line(src,lines[i].pt_top,lines[i].pt_bottom,cv::Scalar (255,0,0),2,cv::LINE_AA);         //LINE_AA 抗锯齿
            //绘制跟踪点
            cv::circle(src,lines[i].pt_track,3.5,cv::Scalar (0,255,0),-1);
#ifdef WRITE
//            cv::putText(src, to_string((int)(lines[i].angle)),lines[i].new_pt_mid,cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),2);
//            cv::putText(src, to_string((int)(lines[i].neighbor_delta_x)),lines[i].new_pt_mid,cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),2);
#endif

        }
    }

    vector<cv::Point2f> CurbDetector::draw_HoughLinesP(cv::Mat & src, vector<cv::Vec4i> lines)
    {
        vector<cv::Point2f >lines_pts;
        cv::Point2f pt1,pt2,pt_low;
        float x1,y1,x2,y2,x0,y0;
        double theta,angle;
        for(size_t i = 0; i < lines.size();i++)
        {
            x1 = lines[i][0];
            y1 = lines[i][1];
            x2 = lines[i][2];
            y2 = lines[i][3];
            x0 =(x1 + x2) / 2;
            y0 =(y1 + y2) / 2;
            pt1=cv::Point2f (x1,y1);
            pt2=cv::Point2f (x2,y2);
            //靠近图像下方的point后放入vector
            if(pt1.y >= y0)
            {
                pt_low =pt1;
                lines_pts.push_back(pt2);
                lines_pts.push_back(pt1);
            } else{
                pt_low = pt2;
                lines_pts.push_back(pt1);
                lines_pts.push_back(pt2);
            }
            angle = atan((pt_low.y - y0)/(pt_low.x -x0));


            cv::line(src,pt1,pt2,cv::Scalar (255,0,0),2,cv::LINE_AA);         //LINE_AA 抗锯齿
#ifdef WRITE
            cv::putText(src, to_string((int)(angle)),cv::Point2f (x0,y0),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),2);
#endif
        }
        return lines_pts;
    }

    vector<cv::Point2f> CurbDetector::draw_HoughLines(cv::Mat & src, vector<cv::Vec2f> lines)
    {
        vector<cv::Point2f >lines_pts;
        float rho,theta;
        double a,b,x0,y0;
        cv::Point2f pt1,pt2;
        for(size_t i = 0; i < lines.size(); i++)
        {
            rho = lines[i][0];
            theta =lines[i][1];
            a = cos(theta);
            b = sin(theta);
            x0 = a * rho;
            y0 = b * rho;
            lines_pts.push_back(cv::Point2f (x0,y0));
            //极坐标转成Point
            pt1 = cv::Point2f (cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
            pt2 = cv::Point2f (cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));
            cv::line(src,pt1,pt2,cv::Scalar (255,0,0),2,cv::LINE_AA);         //LINE_AA 抗锯齿
#ifdef WRITE
            cv::putText(src, to_string((int)theta),cv::Point2f (x0,y0),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,255,255),2);
#endif
        }

        return lines_pts;
    }


    void CurbDetector::show_img()
    {
        //绘制合法直线
        cout<<"legal lines size: "<<lines_.size()<<endl;
//        draw_legal_lines(lines_,src);
        //绘制所有
//        lines_pts = draw_HoughLinesP(this->src,lines);
        draw_target_line(target_goal,src);
        if(target_goal.state == 0)
        {
           cv::putText(src,"lost",cv::Point (20,20),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),2);
        } else if(target_goal.state == 1)
        {
            cv::putText(src,"drop",cv::Point (20,20),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),2);
        } else if(target_goal.state == 2)
        {
            cv::putText(src,"track",cv::Point (20,20),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),2);
        }


        cv::namedWindow("Original",cv::WINDOW_NORMAL);
        cv::imshow("Original",this->src);
        //    cv::waitKey(1);

        cv::namedWindow("Gray",cv::WINDOW_NORMAL);
        cv::imshow("Gray", this->gray_img);
        //    cv::waitKey(1);

        cv::namedWindow("Gauss",cv::WINDOW_NORMAL);
        cv::imshow("Gauss", this->gauss_img);
        //    cv::waitKey(1);

        cv::namedWindow("Canny",cv::WINDOW_NORMAL);
        cv::imshow("Canny", this->canny_img);
        //    cv::waitKey(1);
#ifdef THRE
        cv::namedWindow("Threshold",cv::WINDOW_NORMAL);
        cv::imshow("Threshold", this->thre_img);
        //    cv::waitKey(1);
#endif

        //    cv::namedWindow("Erode",cv::WINDOW_NORMAL);
        //    cv::imshow("Erode", this->erode_img);
        ////    cv::waitKey(1);
        //        cv::namedWindow("Dilate",cv::WINDOW_NORMAL);
        //        cv::imshow("Dilate", this->dilate_img);
        ////    cv::waitKey(1);



        cv::waitKey(1);
    }




    /*
     * index = -1 删除最后同一个target
     */
    void CurbDetector::pop_target(int index)
    {
        if(this->line_targets.empty())
        {
            cerr<<"Failed pop:: the line target vectors is empty!!!"<<endl;
            exit(-1);
        }
        else if(index > this->line_targets.size()-1)
        {
            cerr<<"Failed pop:: the index is overflew!!!"<<endl;
            exit(-1);
        }
        else
        {
            if(index < 0)
            {
                this->line_targets.pop_back();
            } else
            {
                auto iter = this->line_targets.begin();
                for(size_t i = 0; i < index ; i++)
                {
                    iter++;
                }
                this->line_targets.erase(iter);
            }

        }

    }


    //自动检查删除
    void CurbDetector:: insert_target(Target t)
    {
        if(this->line_targets.size() >= TARGETS_SIZE)
        {
            this->line_targets.erase(this->line_targets.begin());
        }
        this->line_targets.push_back(t);
    }














}


