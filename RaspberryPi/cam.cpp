#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main() {
// string pipeline = "libcamerasrc ! videoconvert ! video/x-raw, width=1280, height=720, format=BGR ! appsink";
// VideoCapture cap(pipeline, CAP_GSTREAMER);
//string pipeline = "libcamerasrc ! videoconvert ! video/x-raw, width=1280, height=720, format=BGR ! appsink";
// 수정된 파이프라인: videoscale을 추가하여 해상도 변환을 확실히 처리합니다.
string pipeline = "libcamerasrc ! videoconvert ! videoscale ! video/x-raw, width=1280, height=720, format=BGR ! appsink drop=true";
VideoCapture cap(pipeline, CAP_GSTREAMER);

//VideoCapture cap(0, CAP_V4L2); 
//cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('p', 'B', 'A', 'A'));
cap.set(CAP_PROP_FRAME_WIDTH, 1280);
cap.set(CAP_PROP_FRAME_HEIGHT, 720);
   
   
   if (!cap.isOpened()) {
        cerr << "카메라를 열 수 없습니다!" << endl;
        return -1;
    }


    //cap.set(CAP_PROP_FRAME_WIDTH, 3280);
    //cap.set(CAP_PROP_FRAME_HEIGHT, 2464);
    cap.set(CAP_PROP_BUFFERSIZE, 1);

     Mat frame;
    for(int i = 0; i<10;i++){
        cap >> frame;
    }
    

    int i = 1;
    while(1){
        char c;
        cout << "quit for q: ";

        cin >> c; 
        
        if (c == 'q') break;

        cout << "버퍼 비우는 중..." << endl;


        for (int k = 0; k < 4; k++) {
            cap.grab(); 
        }

        cout << "촬영!" << endl;
        

  
        cap >> frame; 

        if (frame.empty()) {
            cerr << "빈화면" << endl;
            break;
        }
        string filename = "photo_" + to_string(i) + ".jpg";
        imwrite(filename, frame);
        
        cout << " -> " << filename << " 저장 완료" << endl;
        i++;
    }

    cap.release();
    return 0;
}