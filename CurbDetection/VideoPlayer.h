#ifndef CURBDETECTION_VIDEOPLAYER_H
#define CURBDETECTION_VIDEOPLAYER_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
using namespace std;
using namespace cv;


class VideoPlayer
{
public:
    VideoPlayer(string path)
    {
        _videoPath = path;
        _video.open(_videoPath);
        if (!_video.isOpened())
        {
            cout << "\n\n\nFail to open " << path << "...\n\n\n" << endl;
            exit(0);
        }
        _frameCount = _video.get(CAP_PROP_FRAME_COUNT);
        cout<<"Open "<<path <<" successfully!"<<endl;
        cout << "\nvideo total frame count:" << _frameCount << endl;
        // waitKey(0);
        Mat VideoControl(50, 1200, CV_8UC1, (uchar)0);
        imshow("VideoControl", VideoControl);
//        重置帧数
//        _frameCount = 15000;
//
        this->_controlRate = 1;
        createTrackbar("ControlBar", "VideoControl", &this->_controlRate, this->_frameCount);
        // createTrackbar("ControlBar", "VideoControl", nullptr, 1500);
        waitKey(0);
    }

    void  playOneFrame(vector<Mat>& Mats) {
        //cout<<"isgetFrame1 "<<isgetframe<<" rate1 "<<_controlRate<<endl;
        if (!isgetframe) {
            _video.set(CAP_PROP_POS_FRAMES, this->_controlRate);
            //cout<<"CAP_PROP_POS_FRAMES02 :"<<CAP_PROP_POS_FRAMES<<"  _controlRate02 :"<< this->_controlRate<<endl;
            _video.read(_frame);
            this->_controlRate++;
            setTrackbarPos("ControlBar", "VideoControl", this->_controlRate);
            // resize(_frame, _frame, Size(), 0.5, 0.5);
        }
        this->isgetframe = false;
        //cout<<"isgetFrame2 "<<isgetframe<<" rate2 "<<_controlRate<<endl;
    }

    void playBackOneFrame(vector<Mat>& Mats)
    {
        if (!isgetframe) {
            _video.set(CAP_PROP_POS_FRAMES, this->_controlRate);
            //cout<<"CAP_PROP_POS_FRAMES02 :"<<CAP_PROP_POS_FRAMES<<"  _controlRate02 :"<< this->_controlRate<<endl;
            _video.read(_frame);
            this->_controlRate --;
            setTrackbarPos("ControlBar", "VideoControl", this->_controlRate);
            // resize(_frame, _frame, Size(), 0.5, 0.5);
        }
        this->isgetframe = false;
    }


    void  play(vector<Mat>& Mats)
    {
        //cout<<"Forward"<<" rate  "<<_controlRate<<endl;
        playOneFrame(Mats);
    }

    void playBack(vector<Mat>& Mats)
    {
        //cout<<"Back"<<" rate  "<<_controlRate<<endl;
        playBackOneFrame(Mats);
    }

    void getFrame(Mat &output)
    {
        //cout<<"isgetFrame0 "<<isgetframe<<" rate0 "<<_controlRate<<endl;
        if (!isgetframe)
        {
            this->_controlRate = getTrackbarPos("ControlBar", "VideoControl");

            _video.set(CAP_PROP_POS_FRAMES, this->_controlRate);
            // _video.set(CAP_PROP_POS_FRAMES, this->_controlRate);
            //cout<<"CAP_PROP_POS_FRAMES01 :"<<CAP_PROP_POS_FRAMES<<"  _controlRate01 :"<< this->_controlRate<<endl;
            _video.read(_frame);
            this->_controlRate++;
            setTrackbarPos("ControlBar", "VideoControl", this->_controlRate);
            // resize(_frame, _frame, Size(), 0.5, 0.5);
            output = _frame.clone();
            isgetframe = true;
        }
    }

private:
    string _videoPath;
    VideoCapture _video;
    Mat _frame;
    int _controlRate = 1;
    int _frameCount;
    bool isgetframe = false;
    int mode = 1;
    int getKey;
};



#endif //CURBDETECTION_VIDEOPLAYER_H
