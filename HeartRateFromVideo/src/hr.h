// Helper declarations and constants for HeartRateFromVideo
#pragma once

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <memory>
#include <vector>

constexpr int BYTES_PER_PIXEL = 3;             // RGB packed
constexpr double DEFAULT_FALLBACK_FPS = 30.0;  // used for duration calculation fallback
// Sampling rate limits: 15 Hz satisfies Nyquist for 4 Hz heart rate + safety margin; 60 Hz prevents excessive FFT sizes
constexpr double MIN_SAMPLING_RATE = 15.0;   // Hz for resampling
constexpr double MAX_SAMPLING_RATE = 60.0;   // Hz for resampling
constexpr double SEARCH_MIN_HZ = 0.66;       // 40 BPM
constexpr double SEARCH_MAX_HZ = 3.00;       // 180 BPM
constexpr double EPSILON_SMALL = 1e-12;      // small value to avoid div by zero
constexpr double SECONDS_PER_MINUTE = 60.0;  // conversion factor from Hz to BPM

// Forehead ROI calculation and storage
class Forehead {
   public:
    static constexpr double WIDTH = 1080.0;
    static constexpr double HEIGHT = 720.0;
    static constexpr double X_START_FRAC = 380.0 / WIDTH;   // left fraction of width
    static constexpr double X_END_FRAC = 545.0 / WIDTH;     // right fraction of width
    static constexpr double Y_START_FRAC = 156.0 / HEIGHT;  // top fraction of height
    static constexpr double Y_END_FRAC = 224.0 / HEIGHT;    // bottom fraction of height

    // No default constructor - must provide sample with caps
    Forehead() = delete;

    // Constructor calculates ROI from sample caps
    explicit Forehead(GstSample* sample);
    ~Forehead() = default;
    Forehead(const Forehead&) = delete;
    Forehead& operator=(const Forehead&) = delete;

    // Frame dimensions (cached from caps)
    int frameWidth;
    int frameHeight;

    // ROI bounds (public for direct access)
    int xStart;
    int xEnd;
    int yStart;
    int yEnd;
};

// Process a GstSample pulled from appsink. Returns true if processed and
// appends timestamp (seconds) and average green to the provided vectors.
// Parameter naming uses camelCase and pointer/reference tokens are attached
// to the type (e.g., GstSample* sample).
bool processSampleForGreen(GstSample* sample, size_t frameIndex, std::shared_ptr<Forehead>& forehead,
                           std::vector<double>& outTimes, std::vector<double>& outGreen);

// Given collected times and green signal, resample/interpolate to a uniform
// grid and estimate BPM using gst-fft. Returns estimated BPM (>0) or 0 on
// failure.
double resampleAndEstimateBpm(const std::vector<double>& sigTimes, const std::vector<double>& sigGreen);
