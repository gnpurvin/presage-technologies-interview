#include "hr.h"

#include <gst/app/gstappsink.h>
#include <gst/fft/gstfft.h>
#include <gst/fft/gstfftf64.h>
#include <gst/gst.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

// Forehead constructor - calculates ROI from sample caps
Forehead::Forehead(GstSample* sample) {
    if (!sample) {
        throw std::runtime_error("Forehead constructor: null sample");
    }

    // Extract frame dimensions from sample caps
    GstCaps* sampleCaps = gst_sample_get_caps(sample);
    if (!sampleCaps) {
        throw std::runtime_error("Forehead constructor: sample has no caps");
    }

    GstStructure* capsStruct = gst_caps_get_structure(sampleCaps, 0);
    if (!capsStruct) {
        throw std::runtime_error("Forehead constructor: caps structure is null");
    }

    gint frameWidth = 0;
    gint frameHeight = 0;
    gst_structure_get_int(capsStruct, "width", &frameWidth);
    gst_structure_get_int(capsStruct, "height", &frameHeight);

    // Validate frame dimensions
    if (frameWidth <= 0 || frameHeight <= 0) {
        throw std::runtime_error("Forehead constructor: invalid frame dimensions");
    }

    // Store frame dimensions for later use
    this->frameWidth = frameWidth;
    this->frameHeight = frameHeight;

    // Compute forehead ROI in pixel coordinates using class constants
    int tempXStart = static_cast<int>((frameWidth * X_START_FRAC));
    int tempXEnd = static_cast<int>((frameWidth * X_END_FRAC));
    int tempYStart = static_cast<int>((frameHeight * Y_START_FRAC));
    int tempYEnd = static_cast<int>((frameHeight * Y_END_FRAC));

    // Clamp bounds to avoid out-of-bounds access
    xStart = std::max(0, tempXStart);
    yStart = std::max(0, tempYStart);
    xEnd = std::min(frameWidth, tempXEnd);
    yEnd = std::min(frameHeight, tempYEnd);

    // Validate ROI is not empty
    if (xStart >= xEnd || yStart >= yEnd) {
        throw std::runtime_error("Forehead ROI is empty or invalid");
    }
}

// processSampleForGreen
// Maps the GstBuffer contained in 'sample', computes the average green
// pixel value within a forehead ROI. Returns true on success.
bool processSampleForGreen(GstSample* sample, size_t frameIndex, std::shared_ptr<Forehead>& forehead,
                           std::vector<double>& outTimes, std::vector<double>& outGreen) {
    if (!sample) {
        std::cout << "processSampleForGreen: null sample\n";
        return false;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        std::cout << "processSampleForGreen: sample has no buffer\n";
        return false;
    }

    // Obtain frame width/height from sample caps if forehead not yet calculated
    if (!forehead) {
        // Create forehead ROI object (only done once)
        try {
            forehead = std::make_shared<Forehead>(sample);
        } catch (const std::runtime_error& e) {
            std::cout << "processSampleForGreen: " << e.what() << "\n";
            return false;
        }
    }

    // Get timestamp in seconds; return false if unavailable
    GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (pts == GST_CLOCK_TIME_NONE) {
        std::cout << "processSampleForGreen: buffer has no PTS timestamp\n";
        return false;
    }

    // Process RGB packed data within forehead ROI
    GstMapInfo mapInfo;
    if (!gst_buffer_map(buffer, &mapInfo, GST_MAP_READ)) {
        std::cout << "processSampleForGreen: failed to map buffer\n";
        return false;
    }

    const guint8* pixelData = mapInfo.data;
    uint64_t greenSum = 0;
    size_t pixelCount = 0;

    // Use cached frame width for stride calculation
    size_t rowStride = (static_cast<size_t>(forehead->frameWidth) * BYTES_PER_PIXEL);
    for (size_t rowNum = forehead->yStart; rowNum < forehead->yEnd; ++rowNum) {
        const guint8* rowData = pixelData + (rowNum * rowStride);
        for (int colNum = forehead->xStart; colNum < forehead->xEnd; ++colNum) {
            int pixelOffset = (colNum * BYTES_PER_PIXEL);
            uint8_t g = rowData[pixelOffset + 1];
            greenSum += g;
            ++pixelCount;
        }
    }

    double averageGreen = (static_cast<double>(greenSum) / static_cast<double>(pixelCount));
    std::cout << "Frame " << frameIndex << ": avg_green=" << averageGreen << "\n";

    outTimes.push_back((static_cast<double>(pts) / GST_SECOND));
    outGreen.push_back(averageGreen);

    gst_buffer_unmap(buffer, &mapInfo);
    return true;
}

// resampleAndEstimateBpm
// Resamples the collected non-uniformly spaced signal to a uniform grid via
// linear interpolation, removes the DC mean, computes the FFT using gst-fft
// and searches for the peak within a physiological frequency range.
double resampleAndEstimateBpm(const std::vector<double>& sigTimes, const std::vector<double>& sigGreen) {
    if ((sigTimes.size() < 2) || (sigTimes.size() != sigGreen.size())) {
        std::cout << "resampleAndEstimateBpm: insufficient or mismatched signal data\n";
        return 0.0;
    }

    // Calculate signal duration from timestamps
    double startTime = sigTimes.front();
    double endTime = sigTimes.back();
    double duration = (endTime - startTime);
    if (duration <= 0.0) {
        duration = (static_cast<double>(sigGreen.size()) / DEFAULT_FALLBACK_FPS);
    }

    // Calculate target sampling frequency and clamp to reasonable range (15-60 Hz)
    double measuredFps = (static_cast<double>(sigGreen.size()) / duration);
    double targetSampleFreq = std::max(MIN_SAMPLING_RATE, std::min(MAX_SAMPLING_RATE, measuredFps));

    // Find the next power of 2 for FFT size (required by gst-fft)
    int fourierSize = 1;
    while (fourierSize < static_cast<int>((targetSampleFreq * duration))) {
        fourierSize <<= 1;  // next power of two
    }

    // Create uniform time grid and signal arrays
    std::vector<double> uniformTimes(fourierSize);
    std::vector<double> uniformSignal(fourierSize);

    // Generate uniform time samples over the signal duration
    for (int i = 0; i < fourierSize; ++i) {
        uniformTimes[i] = startTime + ((duration * i) / static_cast<double>(fourierSize));
    }

    // Resample original signal onto uniform grid using linear interpolation
    for (int i = 0; i < fourierSize; ++i) {
        double uniformTime = uniformTimes[i];
        auto lowerBound = std::lower_bound(sigTimes.begin(), sigTimes.end(), uniformTime);
        if (lowerBound == sigTimes.begin()) {
            uniformSignal[i] = sigGreen.front();
        } else if (lowerBound == sigTimes.end()) {
            uniformSignal[i] = sigGreen.back();
        } else {
            // Linear interpolation between two adjacent samples
            int idx = static_cast<int>(lowerBound - sigTimes.begin());
            int i0 = (idx - 1);
            int i1 = idx;
            double t0 = sigTimes[i0];
            double t1 = sigTimes[i1];
            double v0 = sigGreen[i0];
            double v1 = sigGreen[i1];
            double alpha = ((uniformTime - t0) / (t1 - t0 + EPSILON_SMALL));
            uniformSignal[i] = (v0 * (1.0 - alpha)) + (v1 * alpha);
        }
    }

    // Remove DC component (subtract mean) to isolate oscillations
    double meanValue = 0.0;
    for (double v : uniformSignal) {
        meanValue += v;
    }
    meanValue /= static_cast<double>(uniformSignal.size());
    for (double& v : uniformSignal) {
        v -= meanValue;
    }

    // Compute FFT to convert signal to frequency domain
    GstFFTF64* fft = gst_fft_f64_new(fourierSize, FALSE);
    if (fft == nullptr) {
        return 0.0;
    }

    std::vector<GstFFTF64Complex> freqBins((fourierSize / 2) + 1);
    gst_fft_f64_fft(fft, uniformSignal.data(), freqBins.data());

    // Calculate frequency bin indices for physiological range (0.5-4 Hz = 30-240 BPM)
    double Fs = (static_cast<double>(fourierSize) / duration);  // sampling frequency in Hz
    int kmin = std::max(1, static_cast<int>(std::floor((SEARCH_MIN_HZ * static_cast<double>(fourierSize)) / Fs)));
    int kmax = std::min((fourierSize / 2) - 1,
                        static_cast<int>(std::ceil((SEARCH_MAX_HZ * static_cast<double>(fourierSize)) / Fs)));

    // Find frequency bin with maximum power (dominant heart rate)
    double bestPower = 0.0;
    int bestK = kmin;
    for (int k = kmin; k <= kmax; ++k) {
        double re = freqBins[k].r;
        double im = freqBins[k].i;
        double power = (re * re) + (im * im);  // magnitude squared
        if (power > bestPower) {
            bestPower = power;
            bestK = k;
        }
    }

    gst_fft_f64_free(fft);

    // Convert peak frequency to BPM estimate
    double peakFreqHz = (static_cast<double>(bestK) * Fs) / static_cast<double>(fourierSize);
    double estimatedBpm = (peakFreqHz * 60.0);
    return estimatedBpm;
}
