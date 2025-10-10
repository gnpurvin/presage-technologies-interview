#include "pipeline.h"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <iostream>

GstElement* buildPipeline(const std::string& filePath, GstElement** outAppsink) {
    GstElement* pipeline = gst_pipeline_new("hr-pipeline");
    // Read video file from disk
    GstElement* source = gst_element_factory_make("filesrc", "source");
    // Auto-detect and decode video format
    GstElement* decode = gst_element_factory_make("decodebin", "decode");
    // Convert to standard RGB format
    GstElement* convert = gst_element_factory_make("videoconvert", "convert");
    // Isolate green channel for HR detection
    GstElement* chromahold = gst_element_factory_make("chromahold", "chromahold");
    // Force caps negotiation to ensure dimensions are always available in samples
    GstElement* capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    // Ensure compatible format for appsink
    GstElement* convert2 = gst_element_factory_make("videoconvert", "convert2");
    // Pull video frames into application
    GstElement* appsink = gst_element_factory_make("appsink", "appsink");

    if (!pipeline || !source || !decode || !convert || !chromahold || !capsfilter || !convert2 || !appsink) {
        std::cout << "Failed to create GStreamer elements. Is gstreamer installed?" << std::endl;
        if (pipeline) {
            gst_object_unref(pipeline);
        }
        return nullptr;
    }

    g_object_set(G_OBJECT(source), "location", filePath.c_str(), nullptr);

    /* Configure chromahold to isolate green channel for heart rate detection */
    g_object_set(G_OBJECT(chromahold), "target-r", 0, "target-g", 255, "target-b", 0, "tolerance", 30, nullptr);

    /* Configure capsfilter to enforce specific video format and ensure caps are available */
    GstCaps* filterCaps = gst_caps_from_string("video/x-raw");
    g_object_set(G_OBJECT(capsfilter), "caps", filterCaps, nullptr);
    gst_caps_unref(filterCaps);

    /* Configure appsink to be pull-based and non-blocking */
    g_object_set(G_OBJECT(appsink), "emit-signals", FALSE, "sync", FALSE, nullptr);

    gst_bin_add_many(GST_BIN(pipeline), source, decode, convert, chromahold, capsfilter, convert2, appsink, NULL);

    if (!gst_element_link(source, decode)) {
        std::cout << "Failed to link source -> decodebin" << std::endl;
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (!gst_element_link(convert, chromahold)) {
        std::cout << "Failed to link videoconvert -> chromahold" << std::endl;
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (!gst_element_link(chromahold, capsfilter)) {
        std::cout << "Failed to link chromahold -> capsfilter" << std::endl;
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (!gst_element_link(capsfilter, convert2)) {
        std::cout << "Failed to link capsfilter -> videoconvert2" << std::endl;
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (!gst_element_link(convert2, appsink)) {
        std::cout << "Failed to link videoconvert2 -> appsink" << std::endl;
        gst_object_unref(pipeline);
        return nullptr;
    }

    /* decodebin will create pads dynamically; connect to pad-added to link to
     * convert */
    g_signal_connect(decode, "pad-added", G_CALLBACK(onPadAdded), convert);

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        std::cout << "Unable to set the pipeline to the playing state." << std::endl;
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (outAppsink) {
        *outAppsink = appsink;
    }

    return pipeline;
}

void onPadAdded(GstElement* src, GstPad* pad, gpointer data) {
    GstElement* sinkElem = GST_ELEMENT(data);
    GstPad* sinkPad = gst_element_get_static_pad(sinkElem, "sink");

    if (!sinkPad) {
        std::cout << "Failed to get static sink pad from element " << GST_ELEMENT_NAME(sinkElem) << std::endl;
        return;
    }

    // pad is already linked
    if (gst_pad_is_linked(sinkPad)) {
        std::cout << "Sink pad from element " << GST_ELEMENT_NAME(sinkElem) << " is already linked." << std::endl;
        gst_object_unref(sinkPad);
        return;
    }

    GstCaps* caps = gst_pad_get_current_caps(pad);
    if (!caps) {
        caps = gst_pad_query_caps(pad, nullptr);
    }

    if (caps) {
        GstStructure* str = gst_caps_get_structure(caps, 0);
        const gchar* name = gst_structure_get_name(str);
        if (g_str_has_prefix(name, "video/")) {
            GstPadLinkReturn ret = gst_pad_link(pad, sinkPad);
            if (ret != GST_PAD_LINK_OK) {
                std::cout << "Failed to link decodebin pad to videoconvert sink: " << ret << std::endl;
            }
        }

        gst_caps_unref(caps);
    }

    gst_object_unref(sinkPad);
}

bool pollBusForMessages(GstBus* bus) {
    // Poll the bus with a short timeout to avoid blocking the appsink loop.
    GstMessage* msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
                                                 static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    bool ret = false;
    if (!msg) {
        // no message
        return ret;
    }
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError* err = nullptr;
            gchar* debugInfo = nullptr;
            gst_message_parse_error(msg, &err, &debugInfo);
            std::cout << "Error received from element " << GST_OBJECT_NAME(msg->src) << ": " << err->message
                      << std::endl;
            std::cout << "Debugging information: " << (debugInfo ? debugInfo : "none") << std::endl;
            g_clear_error(&err);
            g_free(debugInfo);
            ret = true;  // signal caller to break
            break;
        }
        case GST_MESSAGE_EOS: {
            std::cout << "End-Of-Stream reached." << std::endl;
            ret = true;  // signal caller to break
            break;
        }
        default: {
            break;
        }
    }
    gst_message_unref(msg);
    return ret;  // no relevant message
}
