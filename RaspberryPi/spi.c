#include "protocol.h"

// SPI 장치 파일 경로 (CE0 핀 사용 시)
static const char *device = "/dev/spidev0.0";

int main() {
    int fd;
    int ret;
    
    // SPI 설정 변수
    uint8_t mode = 0;         // SPI Mode 0 (CPOL=0, CPHA=0)
    uint8_t bits = 8;         // 8 bits per word
    uint32_t speed = 500000;  // 500`kHz (속도)
    
    // 데이터 버퍼
    Packet rx_data;
    uint8_t tx_buf[sizeof(Packet)] = {0, }; // 더미 데이터 (0x00)
    uint8_t rx_buf[sizeof(Packet)] = {0, }; // 수신 데이터 버퍼

    // 2. SPI 장치 열기
    fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("SPI 장치 열기 실패");
        return -1;
    }

    // 3. SPI 모드 및 속도 설정
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("SPI 통신 시작 (크기: %d bytes)...\n", sizeof(Packet));

    // 4. 전송 구조체 설정
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf, // 보낼 더미 데이터 주소
        .rx_buf = (unsigned long)rx_buf, // 받을 버퍼 주소
        .len = sizeof(Packet),           // 주고받을 길이 (8바이트)
        .speed_hz = speed,
        .bits_per_word = bits,
        .delay_usecs = 0,
    };

    while (1) {
        // 5. 트랜잭션 실행 (송수신 동시 발생)
        // 라즈베리파이가 tx_buf(0x00)를 쏘면서 rx_buf에 STM32 데이터를 채워옴
        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1) {
            perror("통신 오류");
            break;
        }

        // 6. 바이트 배열 -> 구조체로 변환 (메모리 복사)
        memcpy(&rx_data, rx_buf, sizeof(Packet));

        // 7. 결과 출력
        printf("속도: %d, 과속여부: %d\n", rx_data.speed, rx_data.is_overspeed);

        if (rx_data.is_overspeed == 1) {
            printf(">>> 과속차량\n");
            // 여기에 이미지 수신 함수 호출 등을 추가하면 됨
        }

        usleep(100000); // 0.1초 대기
    }

    close(fd);
    return 0;
}
