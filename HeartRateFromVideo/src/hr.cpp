#include "hr.h"

#include <gst/app/gstappsink.h>
#include <gst/fft/gstfft.h>
#include <gst/fft/gstfftf64.h>
#include <gst/gst.h>

#include <algorithm>
#include <iostream>

// processSampleForGreen
// Maps the GstBuffer contained in 'sample', computes the average green
// pixel value within a forehead ROI (or whole frame if dimensions are
// output vectors. Returns true on success.
bool processSampleForGreen(GstSample* sample, size_t frameIndex,
                           double fallbackRate, std::vector<double>& outTimes,
                           std::vector<double>& outGreen) {
    if (sample == nullptr) {
        return false;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (buffer == nullptr) {
        return false;
    }

    GstClockTime pts = GST_BUFFER_PTS(buffer);
    double timestampSeconds = 0.0;
    if (pts == GST_CLOCK_TIME_NONE) {
        timestampSeconds = -1.0;
    } else {
        timestampSeconds = static_cast<double>(pts) / GST_SECOND;
    }

    GstMapInfo mapInfo;
    if (!gst_buffer_map(buffer, &mapInfo, GST_MAP_READ)) {
        return false;
    }

    const guint8* pixelData = mapInfo.data;

    // Obtain frame width/height from sample caps if available.
    gint frameWidth = 0;
    gint frameHeight = 0;
    GstCaps* sampleCaps = gst_sample_get_caps(sample);
    if (sampleCaps != nullptr) {
        GstStructure* capsStruct = gst_caps_get_structure(sampleCaps, 0);
        gst_structure_get_int(capsStruct, "width", &frameWidth);
        gst_structure_get_int(capsStruct, "height", &frameHeight);
    }

    // Compute forehead ROI in pixel coordinates using named constants
    int roiX0 = 0;
    int roiX1 = 0;
    int roiY0 = 0;
    int roiY1 = 0;
    if (frameWidth > 0 && frameHeight > 0) {
        roiX0 = static_cast<int>(frameWidth * FOREHEAD_X_START_FRAC);
        roiX1 = static_cast<int>(frameWidth * FOREHEAD_X_END_FRAC);
        roiY0 = static_cast<int>(frameHeight * FOREHEAD_Y_START_FRAC);
        roiY1 = static_cast<int>(frameHeight * FOREHEAD_Y_END_FRAC);

        if (roiX0 < 0) {
            roiX0 = 0;
        }

        if (roiY0 < 0) {
            roiY0 = 0;
        }

        if (roiX1 > frameWidth) {
            roiX1 = frameWidth;
        }

        if (roiY1 > frameHeight) {
            roiY1 = frameHeight;
        }
    }

    uint64_t greenSum = 0;
    size_t pixelCount = 0;

    // If dimensions look valid and buffer size is sufficient assume RGB packed
    if ((frameWidth > 0) && (frameHeight > 0) &&
        (mapInfo.size >= static_cast<size_t>(frameWidth) *
                             static_cast<size_t>(frameHeight) *
                             BYTES_PER_PIXEL)) {
        size_t rowStride = static_cast<size_t>(frameWidth) * BYTES_PER_PIXEL;
        for (int yy = roiY0; yy < roiY1; ++yy) {
            const guint8* rowPtr =
                pixelData + static_cast<size_t>(yy) * rowStride;
            for (int xx = roiX0; xx < roiX1; ++xx) {
                greenSum += rowPtr[xx * BYTES_PER_PIXEL + 1];
                ++pixelCount;
            }
        }
    } else {
        // Fallback: average whole frame (if possible).
        size_t pixels = (mapInfo.size >= static_cast<size_t>(BYTES_PER_PIXEL))
                            ? (mapInfo.size / BYTES_PER_PIXEL)
                            : 0;
        for (size_t i = 0; i < pixels; ++i) {
            greenSum += pixelData[i * BYTES_PER_PIXEL + 1];
        }
        pixelCount = pixels;
    }

    double averageGreen = 0.0;
    if (pixelCount > 0) {
        averageGreen =
            static_cast<double>(greenSum) / static_cast<double>(pixelCount);
    }

    std::cout << "Frame " << frameIndex << ": avg_green=" << averageGreen
              << "\n";

    // Timestamp fallback uses the provided fallbackRate (e.g., 30 FPS)
    if (timestampSeconds >= 0.0) {
        outTimes.push_back(timestampSeconds);
    } else {
        outTimes.push_back(static_cast<double>(frameIndex) / fallbackRate);
    }

    outGreen.push_back(averageGreen);

    gst_buffer_unmap(buffer, &mapInfo);
    return true;
}

// resampleAndEstimateBpm
// Resamples the collected non-uniformly spaced signal to a uniform grid via
// linear interpolation, removes the DC mean, computes the FFT using gst-fft
// and searches for the peak within a physiological frequency range.
double resampleAndEstimateBpm(const std::vector<double>& sigTimes,
                              const std::vector<double>& sigGreen) {
    if ((sigTimes.size() < 2) || (sigTimes.size() != sigGreen.size())) {
        return 0.0;
    }

    double startTime = sigTimes.front();
    double endTime = sigTimes.back();
    double duration = endTime - startTime;
    if (duration <= 0.0) {
        duration = static_cast<double>(sigGreen.size()) / DEFAULT_FALLBACK_FPS;
    }

    // Estimate instantaneous frame rate and clamp to reasonable resampling
    // rates.
    double measuredFps = static_cast<double>(sigGreen.size()) / duration;
    double targetFs =
        std::max(MIN_SAMPLING_RATE, std::min(MAX_SAMPLING_RATE, measuredFps));

    int N = 1;
    while (N < static_cast<int>(targetFs * duration)) {
        N <<= 1;  // next power of two
    }

    std::vector<double> uniformTimes(N);
    std::vector<double> uniformSignal(N);

    for (int i = 0; i < N; ++i) {
        uniformTimes[i] = startTime + (duration * i) / static_cast<double>(N);
    }

    // Linear interpolation onto the uniform grid.
    for (int i = 0; i < N; ++i) {
        double t = uniformTimes[i];
        auto it = std::lower_bound(sigTimes.begin(), sigTimes.end(), t);
        if (it == sigTimes.begin()) {
            uniformSignal[i] = sigGreen.front();
        } else if (it == sigTimes.end()) {
            uniformSignal[i] = sigGreen.back();
        } else {
            int idx = static_cast<int>(it - sigTimes.begin());
            int i0 = idx - 1;
            int i1 = idx;
            double t0 = sigTimes[i0];
            double t1 = sigTimes[i1];
            double v0 = sigGreen[i0];
            double v1 = sigGreen[i1];
            double alpha = (t - t0) / (t1 - t0 + EPSILON_SMALL);
            uniformSignal[i] = v0 * (1.0 - alpha) + v1 * alpha;
        }
    }

    // Remove DC component (mean) from the signal.
    double meanValue = 0.0;
    for (double v : uniformSignal) {
        meanValue += v;
    }
    meanValue /= static_cast<double>(uniformSignal.size());
    for (double& v : uniformSignal) {
        v -= meanValue;
    }

    // Compute FFT using gst-fft
    GstFFTF64* fft = gst_fft_f64_new(N, FALSE);
    if (fft == nullptr) {
        return 0.0;
    }

    std::vector<GstFFTF64Complex> freqBins(N / 2 + 1);
    gst_fft_f64_fft(fft, uniformSignal.data(), freqBins.data());

    double Fs = static_cast<double>(N) / duration;  // sampling frequency in Hz
    int kmin = std::max(1, static_cast<int>(std::floor(
                               SEARCH_MIN_HZ * static_cast<double>(N) / Fs)));
    int kmax = std::min(
        N / 2 - 1, static_cast<int>(
                       std::ceil(SEARCH_MAX_HZ * static_cast<double>(N) / Fs)));

    double bestPower = 0.0;
    int bestK = kmin;
    for (int k = kmin; k <= kmax; ++k) {
        double re = freqBins[k].r;
        double im = freqBins[k].i;
        double power = re * re + im * im;
        if (power > bestPower) {
            bestPower = power;
            bestK = k;
        }
    }

    gst_fft_f64_free(fft);

    double peakFreqHz =
        static_cast<double>(bestK) * Fs / static_cast<double>(N);
    double estimatedBpm = peakFreqHz * 60.0;
    return estimatedBpm;
}
