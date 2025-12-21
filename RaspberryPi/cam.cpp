#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0, CAP_V4L2); 

    if (!cap.isOpened()) {
        cerr << "카메라를 열 수 없습니다!" << endl;
        return -1;
    }

    
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    
    cap.set(CAP_PROP_BUFFERSIZE, 1);

    for(int k=0; k<5; k++) {
        Mat tmp;
        cap >> tmp;
    }

    int i = 1;
    while(1){
        char c;
        cout<<"quit for q ";
        cin>>c;
        if (c == 'q') break;

    cout << "촬영 시작" << endl;

        cap.grab();
        Mat frame;
        cap >> frame; 

        if (frame.empty()) {
            cerr << "빈화면" << endl;
            break;
        }

        string filename = "photo_" + to_string(i) + ".jpg";
        imwrite(filename, frame);
        
        cout << filename << " 저장 완료" << endl;
        i++;
        
        waitKey(1000); 
    }

    cap.release();
    return 0;
}