#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>
// gst-fft
#include <gst/fft/gstfft.h>
#include <gst/fft/gstfftf64.h>

#include "hr.h"
#include "pipeline.h"

int main(int argc, char** argv) {
    gst_init(&argc, &argv);

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <video-file>\n";
        return -1;
    }

    std::string filePath = argv[1];

    GstElement* pipeline = nullptr;
    GstElement* appsink = nullptr;
    GstBus* bus = nullptr;

    pipeline = buildPipeline(filePath, &appsink);
    if (!pipeline) {
        // errors printed in buildPipeline
        return -1;
    }

    /* Obtain the bus from the pipeline here so the caller owns the reference */
    bus = gst_element_get_bus(pipeline);
    if (!bus) {
        std::cout << "Failed to get GstBus from pipeline.\n";
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        return -1;
    }

    size_t frameCount = 0;

    GstAppSink* sink = GST_APP_SINK(appsink);
    std::vector<double> sigTimes; /* seconds */
    std::vector<double> sigGreen; /* average green per frame */

    while (true) {
        /* try to pull sample from appsink */
        GstSample* sample = gst_app_sink_try_pull_sample(sink, 500 * GST_MSECOND);
        if (sample) {
            bool ok = processSampleForGreen(sample, frameCount, DEFAULT_FALLBACK_FPS, sigTimes, sigGreen);
            if (!ok) {
                std::cout << "Frame " << frameCount << ": sample processing failed or missing buffer\n";
            }

            ++frameCount;
            gst_sample_unref(sample);
            continue;
        }

        /* No sample available within timeout; check bus for messages */
        if (pollBusForMessages(bus)) {
            break;
        }
    }

    std::cout << "Pulled " << frameCount << " frames. Shutting down..." << std::endl;

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);

    /* If we collected a signal, perform spectral analysis to estimate BPM */
    double bpm = resampleAndEstimateBpm(sigTimes, sigGreen);
    if (bpm > 0.0) {
        std::cout << "Estimated heart rate: " << bpm << " BPM" << std::endl;
    } else {
        std::cout << "BPM estimation failed." << std::endl;
    }

    return 0;
}
