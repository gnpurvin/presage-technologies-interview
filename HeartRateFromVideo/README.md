# HeartRateFromVideo

C++ implementation that extracts heart rate from video using computer vision and signal processing techniques.

## How it Works

1. **Green Channel Extraction**: Analyzes the forehead region for green channel pixel values (heart rate signal is strongest in green channel)
2. **Signal Processing**: Applies linear interpolation, DC removal, and FFT analysis 
3. **Heart Rate Detection**: Finds peak in Power Spectral Density within physiological range (40-180 BPM)

## Prerequisites

This project targets Ubuntu 22.04. Install the following dependencies:

1. Update package lists and install build tools:
```bash
sudo apt update
sudo apt install build-essential pkg-config
```

2. Install GStreamer development headers:
```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools
```

3. Install GStreamer plugins for video format support:
```bash
sudo apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

## Build and Run

```bash
make
./HeartRateFromVideo <video-file>
```

Example:
```bash
./HeartRateFromVideo ../codingtest.mov
```

## BPM Measurement Results

**Measured BPM on codingtest.mov: 47.4465 BPM**

The program processes 1,822 frames and outputs the estimated heart rate based on Fourier analysis of green channel variations in the forehead region.

## Technical Details

- **Pipeline**: `filesrc -> decodebin -> videoconvert -> chromahold -> capsfilter -> videoconvert2 -> appsink`
- **ROI**: Forehead region optimized for heart rate detection
- **Algorithm**: FFT-based power spectral density analysis with physiological constraints
- **Performance**: Optimized with frame dimension caching and single ROI calculation
