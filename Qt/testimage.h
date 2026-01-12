#ifndef TESTIMAGE_H
#define TESTIMAGE_H

#include <QWidget>
#include <opencv2/opencv.hpp>
#include "tesseractocr.h" // OCR 클래스 포함

namespace Ui { class TestImage; }

class TestImage : public QWidget
{
    Q_OBJECT

public:
    explicit TestImage(QWidget *parent = nullptr);
    ~TestImage();

private slots:
    void on_btnLoad_clicked();      // 이미지 불러오기
    void on_btnGray_clicked();      // 그레이스케일 변환
    void on_btnThreshold_clicked(); // 이진화(Threshold)
    void on_btnCanny_clicked();     // 에지 검출(Canny)
    void on_btnOCR_clicked();       // 번호판 인식 실행
    void on_thresholdSlider_valueChanged(int value);  // 슬라이더 값이 변경될 때 호출

private:
    Ui::TestImage *ui;
    TesseractOCR ocr;               // OCR 객체
    cv::Mat originalMat;            // 원본 이미지
    cv::Mat processedMat;           // 전처리된 이미지

    void displayImage(const cv::Mat& img, bool isOriginal); // 화면 표시 함수
    QImage matToQImage(const cv::Mat& mat);                 // 변환 함수
};

#endif
