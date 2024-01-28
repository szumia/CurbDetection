#include <iostream>
#include "VideoPlayer.h"
#include "curbDetect.h"

#define VIDEO_PATH "../Sources/03.avi"


CurbDetect::CurbDetector curbDetector_;
//unsigned long long frame_global = 0;

void receive_and_process_video(string video_path) {
    // cv::VideoCapture vc;
    //TODO::debug video path
    VideoPlayer vp1(video_path);

    vector<Mat> Mats;
    char inputKey ;
    while (true) {

        // cv::Mat src;
        // vc >> src;
        Mat src;
        vp1.getFrame(src);
        if(!src.empty())
        {
            //按s/d暂停   长按S/D一帧一帧播放
            if (inputKey == 'S' || inputKey == 's' || inputKey == 'D' || inputKey == 'd') {
                if (src.empty()) return;
                vector<Mat> Mats = {src};
                vp1.play(Mats);

                //处理
                curbDetector_.curb_detect_steps(src);

                inputKey = waitKey(0);
            } else if (inputKey == 'A' || inputKey == 'a')    //倒退一帧
            {

                int epoch = 3;
                while (epoch > 0) {
                    if (src.empty()) return;
                    vector<Mat> Mats = {src};
                    vp1.playBack(Mats);
                    epoch--;
                }

                //处理
                curbDetector_.curb_detect_steps(src);


                inputKey = waitKey(0);
            } else                                            //连续播放
            {
                inputKey = waitKey(10);
                if (src.empty()) return;
                vector<Mat> Mats = {src};
                vp1.play(Mats);

                //处理
                curbDetector_.curb_detect_steps(src);

            }
        }
    }
}



int main() {
    receive_and_process_video(VIDEO_PATH);
    return 0;
}
