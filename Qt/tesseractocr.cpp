#include "tesseractocr.h"
#include <qdebug.h>

TesseractOCR::TesseractOCR() {
    api = std::make_unique<tesseract::TessBaseAPI>();
}
TesseractOCR::~TesseractOCR() {
    if (api) {
        api->End();
    }
}

bool TesseractOCR::init(const char* language) {
    // Tesseract API 초기화
    // 첫 번째 인자는 'tessdata' 폴더가 포함된 상위 경로입니다.
    const char* datapath = "C:\\Program Files\\Tesseract-OCR\\tessdata";
    if (api->Init(datapath, language, tesseract::OEM_DEFAULT)) {
        qDebug() << "Could not initialize tesseract.";
        return false;
    }

    // 화이트리스트 설정
    api->SetVariable("tessedit_char_whitelist", "0123456789가나다라마거너더러머버서어저고노도로모보소오조구누두루무부수우주바사아자배하허호");

    // 사전 기능 비활성화
    api->SetVariable("load_system_dawg", "F");
    api->SetVariable("load_freq_dawg", "F");

    // 패턴 설정 (이 부분이 경고가 난다면 아래 2번 방법을 확인하세요)
    api->SetVariable("user_patterns_str", "\\d\\d\\c\\d\\d\\d\\d\n\\d\\d\\d\\c\\d\\d\\d\\d");

    // 페이지 분할 모드 설정 (한 줄 인식)
    api->SetPageSegMode(tesseract::PSM_SINGLE_LINE);

    return true;
}

std::string TesseractOCR::recognize(const cv::Mat& inputImage) {
    if (inputImage.empty()) {
        return "";
    }

    cv::Mat grayImage;

    // Tesseract는 Grayscale 이미지를 선호하므로 변환
    if (inputImage.channels() == 3) {
        cv::cvtColor(inputImage, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = inputImage;
    }

    // OpenCV Mat을 Tesseract API로 전달
    // bytes_per_pixel: 그레이스케일이므로 1
    // bytes_per_line: grayImage.step
    api->SetImage(grayImage.data, grayImage.cols, grayImage.rows, 1, grayImage.step);

    // 인식 수행 및 결과 가져오기
    // GetUTF8Text()는 내부적으로 Recognize()를 호출합니다.
    char* outText = api->GetUTF8Text();
    std::string result = "";

    if (outText) {
        result = std::string(outText);

        // 줄바꿈 문자 제거 (한 줄 인식이지만 결과 뒤에 \n이 붙을 수 있음)
        result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());

        delete[] outText; // 메모리 해제 필수
    }

    // 신뢰도(Confidence)가 너무 낮으면 빈 문자열 반환 (선택 사항)
    // int meanConf = api->MeanTextConf();
    // if (meanConf < 50) return "";

    return result;
}
