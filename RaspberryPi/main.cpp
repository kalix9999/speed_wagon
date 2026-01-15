#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <time.h>
#include <wiringPi.h>
#include <opencv2/opencv.hpp>
#include <mosquitto.h>
#include <softPwm.h>
#include <endian.h>

#include "protocol.h"

#define PORT 8080
#define SPI_DEVICE "/dev/spidev0.0"
#define BUTTON_PIN 0
#define SERVO_PIN 4 



pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_cond  = PTHREAD_COND_INITIALIZER;

int keep_running = 1;
int trigger_overspeed = 0;
int trigger_button = 0;
int shared_current_speed = 0;


void set_servo_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    int duty = 5 + (angle * 20 / 180);
    softPwmWrite(SERVO_PIN, duty);
}


void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    int angle = atoi((char *)msg->payload);
    set_servo_angle(angle);
}

void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {

        mosquitto_subscribe(mosq, NULL, "cam/servo/pan", 0);
        printf(">> [MQTT] 브로커 연결 성공! 구독 시작\n");
    } else {
        printf(">> [MQTT] 연결 실패 (Error: %d)\n", rc);
    }
}


void* spi_monitor_thread(void* arg)
{
    int fd;
    uint8_t mode = 0, bits = 8;
    uint32_t speed = 500000;
    Packet rx_data;
    uint8_t tx_buf[sizeof(Packet)] = {0};
    uint8_t rx_buf[sizeof(Packet)] = {0};

    fd = open(SPI_DEVICE, O_RDWR);
    if (fd < 0) { perror("SPI open fail"); return NULL; } 

    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = sizeof(Packet),
        .speed_hz = speed,
        .bits_per_word = bits
    };

    int prev_overspeed = 0;

    while (keep_running) {
        ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        memcpy(&rx_data, rx_buf, sizeof(Packet));

        pthread_mutex_lock(&g_mutex);
        shared_current_speed = rx_data.speed;

        if (rx_data.is_overspeed && !prev_overspeed) {
            trigger_overspeed = 1;
            pthread_cond_signal(&g_cond);
        }
        pthread_mutex_unlock(&g_mutex);

        prev_overspeed = rx_data.is_overspeed;
        usleep(10000);
    }

    close(fd);
    return NULL;
}


void* button_monitor_thread(void* arg)
{
    pinMode(BUTTON_PIN, INPUT);
    pullUpDnControl(BUTTON_PIN, PUD_DOWN);

    while (keep_running) {
        if (digitalRead(BUTTON_PIN) == 1) {
            pthread_mutex_lock(&g_mutex);
            trigger_button = 1;
            pthread_cond_signal(&g_cond);
            pthread_mutex_unlock(&g_mutex);

            printf(">>> 버튼 눌림\n");
            delay(100);
        }
        delay(50);
    }
    return NULL;
}

void* camera_worker(void* arg)
{
    int sock = *(int*)arg;
    std::string pipeline = "libcamerasrc ! videoconvert ! videoscale ! video/x-raw, width=1280, height=720, format=BGR ! appsink drop=true max-buffers=2 sync=false";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) return NULL;

    

    cv::Mat frame;
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};

    while (keep_running) {
        PacketHeader header;
        pthread_mutex_lock(&g_mutex);
        while (!trigger_button && !trigger_overspeed && keep_running)
            pthread_cond_wait(&g_cond, &g_mutex);
        
        if (!keep_running) { pthread_mutex_unlock(&g_mutex); break; }

        int speed = shared_current_speed;
        int btn = trigger_button;
        trigger_button = trigger_overspeed = 0; 
        pthread_mutex_unlock(&g_mutex);

        for(int i=0; i<5; i++) cap.grab();    
        cap >> frame;
        if (frame.empty()) continue;

        if (btn) {
            time_t now = time(NULL);
            struct tm *t = localtime(&now);
            char filename[128];

            sprintf(filename,
                "capture_%04d%02d%02d_%02d%02d%02d_%dkmh.jpg",
                t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                t->tm_hour, t->tm_min, t->tm_sec, speed);

            cv::imwrite(filename, frame);
            printf("저장됨: %s\n", filename);
        }

        cv::resize(frame, frame, cv::Size(640,480));
        cv::imencode(".jpg", frame, buffer, params);
        uint32_t sum = 0;
        for(int i = 0; i<buffer.size();i++){
            sum+= buffer[i];
        }
        header.speed = htonl(speed);
        header.start = htonl(0x12345);
        header.img_size = htonl(buffer.size());
        header.time = htobe64((uint64_t)time(NULL));
        header.checksum = htonl(sum);

        
`
       
        if (write(sock, &header, sizeof(PacketHeader)) <= 0) break;
        if (write(sock, buffer.data(), buffer.size()) <= 0) break;
    }
    close(sock);
    return NULL;
}


void* mqtt_listener_thread(void* arg)
{
    struct mosquitto *mosq = NULL;


    mosquitto_lib_init();
    

    mosq = mosquitto_new("pi_servo_sub", true, NULL);
    if (!mosq) {
        fprintf(stderr, "Mosquitto init fail\n");
        return NULL;
    }


    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);


    if (mosquitto_connect(mosq, "127.0.0.1", 1883, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT connect fail (Is mosquitto running?)\n");
        return NULL;
    }


    softPwmCreate(SERVO_PIN, 0, 200);
    mosquitto_loop_forever(mosq, -1, 1);

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return NULL;
}


int main()
{

    if (wiringPiSetup() == -1) {
        fprintf(stderr, "WiringPi Setup Fail (Run with sudo?)\n");
        return 1;
    }

    int serv_sock, clnt_sock;
    struct sockaddr_in serv_addr, clnt_addr;
    socklen_t addr_size;

    serv_sock = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(PORT);

    if (bind(serv_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1)
        perror("bind() error");
    
    if (listen(serv_sock, 5) == -1)
        perror("listen() error");

    printf("클라이언트 접속 대기중...\n");

    addr_size = sizeof(clnt_addr);
    clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_addr, &addr_size);
    printf("클라이언트 연결됨!\n");

    pthread_t t1, t2, t3, t4;
    
    pthread_create(&t1, NULL, spi_monitor_thread, NULL);    
    pthread_create(&t2, NULL, button_monitor_thread, NULL); 
    pthread_create(&t3, NULL, camera_worker, &clnt_sock);   
    pthread_create(&t4, NULL, mqtt_listener_thread, NULL);  

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    pthread_join(t3, NULL);
    pthread_join(t4, NULL);

    return 0;
}
