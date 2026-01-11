#ifndef TESSERACTOCR_H
#define TESSERACTOCR_H

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

class TesseractOCR
{
public:
    TesseractOCR();
    ~TesseractOCR();

    // 초기화 함수 (언어 데이터 경로 등 설정)
    bool init(const char* language = "kor");

    // 단일 함수: 이미지를 받아 인식된 번호판 문자열을 반환
    std::string recognize(const cv::Mat& image);

private:
    std::unique_ptr<tesseract::TessBaseAPI> api;

};

#endif // TESSERACTOCR_H
