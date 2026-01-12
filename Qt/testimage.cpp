#include "testimage.h"
#include "ui_testimage.h"
#include <QFileDialog>
#include <QMessageBox>

TestImage::TestImage(QWidget *parent) :
    QWidget(parent), ui(new Ui::TestImage)
{
    ui->setupUi(this);

    // OCR 초기화
    if (!ocr.init("kor")) {
        QMessageBox::warning(this, "OCR 오류", "Tesseract 초기화 실패!");
    }
    // 슬라이더 범위 설정 (0~255)
    ui->thresholdSlider->setRange(0, 255);
    ui->thresholdSlider->setValue(127); // 초기값 설정
}

TestImage::~TestImage() { delete ui; }

// 이미지 불러오기
void TestImage::on_btnLoad_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, "이미지 열기", "", "Images (*.png *.jpg *.bmp)");
    if (fileName.isEmpty()) return;

    // 1. QFile을 사용하여 한글 경로의 파일을 바이너리로 읽음
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) return;

    QByteArray fileData = file.readAll();
    file.close();

    // 2. 읽어온 바이너리 데이터를 std::vector로 변환
    std::vector<char> buffer(fileData.begin(), fileData.end());

    // 3. cv::imdecode를 사용하여 메모리상의 데이터를 Mat으로 변환
    originalMat = cv::imdecode(cv::Mat(buffer), cv::IMREAD_COLOR);

    if (originalMat.empty()) {
        qDebug() << "이미지 디코딩 실패";
        return;
    }

    processedMat = originalMat.clone();
    displayImage(originalMat, true);
    displayImage(processedMat, false);
}

// 전처리: 그레이스케일
void TestImage::on_btnGray_clicked() {
    if (processedMat.empty()) return;
    if (processedMat.channels() == 3) {
        cv::cvtColor(processedMat, processedMat, cv::COLOR_BGR2GRAY);
    }
    displayImage(processedMat, false);
}
// 슬라이더 조작 시 실시간 실행되는 함수
void TestImage::on_thresholdSlider_valueChanged(int value)
{
    if (originalMat.empty()) return;

    // 1. 항상 원본(originalMat)에서 시작해야 해상도 손상 없이 반복 조절 가능
    cv::Mat gray;
    if (originalMat.channels() == 3) {
        cv::cvtColor(originalMat, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = originalMat.clone();
    }

    // 2. 사용자가 선택한 value(슬라이더 값)로 이진화 수행
    // THRESH_OTSU는 빼고 사용자가 정한 값만 적용합니다.
    cv::threshold(gray, processedMat, value, 255, cv::THRESH_BINARY);

    // 3. 결과 화면 업데이트
    displayImage(processedMat, false);

    // 현재 임계값을 라벨에 표시하면 더 직관적입니다.
    ui->lblThresholdValue->setText(QString::number(value));
}
// 전처리: 이진화 (번호판 인식률 향상에 중요)
void TestImage::on_btnThreshold_clicked() {
    if (processedMat.empty()) return;
    if (processedMat.channels() == 3) {
        cv::cvtColor(processedMat, processedMat, cv::COLOR_BGR2GRAY);
    }
    // OTSU 알고리즘을 사용한 자동 임계값 이진화
    cv::threshold(processedMat, processedMat, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    displayImage(processedMat, false);
}

// 전처리: Canny 에지 검출 (외곽선 확인용)
void TestImage::on_btnCanny_clicked() {
    if (processedMat.empty()) return;
    cv::Canny(processedMat, processedMat, 100, 200);
    displayImage(processedMat, false);
}

// OCR 인식 수행
void TestImage::on_btnOCR_clicked() {
    if (processedMat.empty()) return;

    // 전처리된 이미지를 OCR 클래스에 전달
    std::string result = ocr.recognize(processedMat);

    // 결과를 UI의 Label이나 TextBrowser에 표시
    ui->lblOcrResult->setText(QString::fromStdString(result));
}

// 이미지를 QLabel에 맞게 표시하는 유틸리티
void TestImage::displayImage(const cv::Mat& img, bool isOriginal) {
    QImage qimg = matToQImage(img);
    QPixmap pix = QPixmap::fromImage(qimg);

    if (isOriginal)
        ui->lblOriginal->setPixmap(pix.scaled(ui->lblOriginal->size(), Qt::KeepAspectRatio));
    else
        ui->lblProcessed->setPixmap(pix.scaled(ui->lblProcessed->size(), Qt::KeepAspectRatio));
}

// OpenCV Mat을 Qt QImage로 변환
QImage TestImage::matToQImage(const cv::Mat& mat) {
    if (mat.type() == CV_8UC1) {
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
    } else if (mat.type() == CV_8UC3) {
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888);
    }
    return QImage();
}
