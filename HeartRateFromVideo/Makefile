
CC = g++
STD = -std=c++17
OPT = -O2

SRC = src/main.cpp src/hr.cpp src/pipeline.cpp
OUT = HeartRateFromVideo

PKG_CFLAGS := $(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 gstreamer-fft-1.0)
PKG_LIBS := $(shell pkg-config --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-fft-1.0)

all: $(OUT)

$(OUT): $(SRC)
	$(CC) $(STD) $(OPT) $(PKG_CFLAGS) -o $@ $^ $(PKG_LIBS)

clean:
	rm -f $(OUT)

.PHONY: all clean
