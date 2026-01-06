#include "protocol.h"


static const char *device = "/dev/spidev0.0";

int main() {
    int spi_fd;
    int ret;
    
    uint8_t mode = 0;      
    uint8_t bits = 8;      
    uint32_t speed = 500000;  
    

    Packet rx_data;
    uint8_t tx_buf[sizeof(Packet)] = {0, }; 
    uint8_t rx_buf[sizeof(Packet)] = {0, }; 


    spi_fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("SPI 장치 열기 실패");
        return -1;
    }


    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("SPI 통신 시작 (크기: %d bytes)...\n", sizeof(Packet));

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf, 
        .rx_buf = (unsigned long)rx_buf, 
        .len = sizeof(Packet),           
        .speed_hz = speed,
        .bits_per_word = bits,
        .delay_usecs = 0,
    };

    while (1) {
       int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1) {
            perror("통신 오류");
            break;
        }

        
        memcpy(&rx_data, rx_buf, sizeof(Packet));

        printf("속도: %d, 과속여부: %d\n", rx_data.speed, rx_data.is_overspeed);

        if (rx_data.is_overspeed == 1) {
            printf(">>> 과속차량\n");
        }

        usleep(100000); 
    }

    close(spi_fd);
    return 0;
}
