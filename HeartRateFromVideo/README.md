# HeartRateFromVideo

Sample C++ project demonstrating programmatic construction of a GStreamer pipeline that reads a video file and delivers frames to an appsink.


Ubuntu 22.04 (target)

This project targets Ubuntu 22.04. To build and run on that platform, install the following prerequisites and GStreamer development packages:

1. Update package lists and install build tools and pkg-config:

    sudo apt update
    sudo apt install build-essential pkg-config

2. Install GStreamer development headers and tools:

    sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools

3. (Recommended) Install common plugin packages so decoding for popular formats works out-of-the-box:

    sudo apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav

Build and run

   make

Usage

   ./HeartRateFromVideo <video-file>

The program constructs a pipeline programmatically (filesrc -> decodebin -> videoconvert -> appsink) and prints a short log for each pulled frame (size and PTS).

Notes

- This is an instructional minimal example. For production use add robust error handling and a frame-processing implementation.
- If `make` fails with pkg-config errors, verify `pkg-config` is installed and that the packages above completed successfully.

Usage

  ./HeartRateFromVideo <video-file>

The program will print a small log for each pulled frame (size and PTS). It demonstrates how to: filesrc -> decodebin -> videoconvert -> appsink, with dynamic pad linking.

Notes
- This is a minimal example meant for instructional purposes. For production use, add better error handling, thread-safety, and a proper frame processing loop.
