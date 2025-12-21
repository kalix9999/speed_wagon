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


typedef struct __attribute__((packed)){
    int speed;
    bool is_overspeed;
}Packet;
