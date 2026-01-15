#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>



#define PORT 8080

#pragma pack(push, 1)
typedef struct __attribute__((packed)){
    int speed;
    bool is_overspeed;
}Packet;



typedef struct{
  uint32_t start;
  uint32_t img_size;
  uint32_t speed;
  uint64_t time;
  uint32_t checksum;
}PacketHeader

#pragma pack(pop)
