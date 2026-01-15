CC=g++
TARGET=main
SOURCE=main.cpp
OBJS=$(SOURCE:.cpp=.o)

CXXFLAGS = -Wall -O2 $(shell pkg-config --cflags opencv4)
LDFLAGS = $(shell pkg-config --libs opencv4) -lmosquitto -lwiringPi -lpthread


all: $(TARGET) cam

cam:
	$(CC) -o $@ cam.cpp $(CXXFLAGS) $(LDFLAGS)

$(TARGET) : $(OBJS)
	$(CC) -o  $@ $^ $(CXXFLAGS) $(LDFLAGS)

%.o:%.cpp
	$(CC) $(CXXFLAGS) -c $< -o $@



clean:
	rm -rf $(TARGET) $(OBJS) cam *.jpg

