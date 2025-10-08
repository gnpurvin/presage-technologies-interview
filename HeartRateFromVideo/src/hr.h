// Helper declarations and constants for HeartRateFromVideo
#pragma once

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <vector>

constexpr double FOREHEAD_X_START_FRAC = 0.35;  // left fraction of width
constexpr double FOREHEAD_X_END_FRAC = 0.65;    // right fraction of width
constexpr double FOREHEAD_Y_START_FRAC = 0.08;  // top fraction of height
constexpr double FOREHEAD_Y_END_FRAC = 0.22;    // bottom fraction of height

constexpr int BYTES_PER_PIXEL = 3;             // RGB packed
constexpr double DEFAULT_FALLBACK_FPS = 30.0;  // used if frame PTS unavailable
constexpr double MIN_SAMPLING_RATE = 15.0;     // Hz for resampling
constexpr double MAX_SAMPLING_RATE = 60.0;     // Hz for resampling
constexpr int MIN_FRAMES_FOR_EST = 32;         // minimum frames to attempt BPM
constexpr double SEARCH_MIN_HZ = 0.66;         // 40 BPM
constexpr double SEARCH_MAX_HZ = 3.00;         // 180 BPM
constexpr double EPSILON_SMALL = 1e-12;  // small value to avoid div by zero

// Process a GstSample pulled from appsink. Returns true if processed and
// appends timestamp (seconds) and average green to the provided vectors.
// Parameter naming uses camelCase and pointer/reference tokens are attached
// to the type (e.g., GstSample* sample).
bool processSampleForGreen(GstSample* sample, size_t frameIndex,
                           double fallbackRate, std::vector<double>& outTimes,
                           std::vector<double>& outGreen);

// Given collected times and green signal, resample/interpolate to a uniform
// grid and estimate BPM using gst-fft. Returns estimated BPM (>0) or 0 on
// failure.
double resampleAndEstimateBpm(const std::vector<double>& sigTimes,
                              const std::vector<double>& sigGreen);

