#include "tesseractocr.h"

TesseractOCR::TesseractOCR() {
    api = std::make_unique<tesseract::TessBaseAPI>();
}
TesseractOCR::~TesseractOCR() {
    if (api) {
        api->End();
    }
}
bool TesseractOCR::init(const char* language) {
    // 1. 초기화 시 설정할 변수 리스트 준비
    std::vector<std::string> vars_vec;
    std::vector<std::string> vars_values;

    // [패턴] 한국 번호판 규격 (00가0000, 000가0000)
    vars_vec.push_back("user_patterns_str");
    vars_values.push_back("\\d\\d\\c\\d\\d\\d\\d\n\\d\\d\\d\\c\\d\\d\\d\\d");

    // [화이트리스트] 숫자 및 번호판용 한글만 허용
    vars_vec.push_back("tessedit_char_whitelist");
    vars_values.push_back("0123456789"
                          "가나다라마거너더러머버서어저고노도로모보소오조구누두루무부수우주"
                          "바사아자배하허호");

    // [사전 비활성화] 무작위 조합 인식을 위해 사전 보정 기능 끔
    vars_vec.push_back("load_system_dawg");
    vars_values.push_back("F");
    vars_vec.push_back("load_freq_dawg");
    vars_values.push_back("F");

    // 2. Tesseract API 초기화
    // 첫 번째 인자는 'tessdata' 폴더가 포함된 상위 경로입니다.
    const char* datapath = "C:/Program Files/Tesseract-OCR";
    if (api->Init(datapath, language, tesseract::OEM_DEFAULT,
                  nullptr, 0, &vars_vec, &vars_values, false)) {
        fprintf(stderr, "Could not initialize tesseract.\n");
        return false;
    }

    // 3. 페이지 분할 모드 설정 (한 줄 인식)
    // 이 설정은 초기화(Init) 후에 호출해야 안전하게 적용됩니다.
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
