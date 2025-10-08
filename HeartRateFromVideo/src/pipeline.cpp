#include "pipeline.h"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <iostream>

void onPadAdded(GstElement* src, GstPad* pad, gpointer data) {
    GstElement* sinkElem = GST_ELEMENT(data);
    GstPad* sinkPad = gst_element_get_static_pad(sinkElem, "sink");

    if (sinkPad == nullptr) {
        return;
    }

    if (gst_pad_is_linked(sinkPad)) {
        gst_object_unref(sinkPad);
        return;
    }

    GstCaps* caps = gst_pad_get_current_caps(pad);
    if (caps == nullptr) {
        caps = gst_pad_query_caps(pad, nullptr);
    }

    if (caps != nullptr) {
        GstStructure* str = gst_caps_get_structure(caps, 0);
        const gchar* name = gst_structure_get_name(str);
        if (g_str_has_prefix(name, "video/")) {
            GstPadLinkReturn ret = gst_pad_link(pad, sinkPad);
            if (ret != GST_PAD_LINK_OK) {
                g_printerr(
                    "Failed to link decodebin pad to videoconvert sink: %d\n",
                    ret);
            }
        }

        gst_caps_unref(caps);
    }

    gst_object_unref(sinkPad);
}

GstElement* buildPipeline(const char* filePath, GstElement** outAppsink) {
    GstElement* pipeline = gst_pipeline_new("hr-pipeline");
    GstElement* source = gst_element_factory_make("filesrc", "source");
    GstElement* decode = gst_element_factory_make("decodebin", "decode");
    GstElement* convert = gst_element_factory_make("videoconvert", "convert");
    GstElement* chromahold = gst_element_factory_make("chromahold", "chromahold");
    GstElement* convert2 = gst_element_factory_make("videoconvert", "convert2");
    GstElement* appsink = gst_element_factory_make("appsink", "appsink");

    if ((pipeline == nullptr) || (source == nullptr) || (decode == nullptr) ||
        (convert == nullptr) || (chromahold == nullptr) ||
        (convert2 == nullptr) || (appsink == nullptr)) {
        g_printerr("Failed to create GStreamer elements. Is gstreamer installed?\n");
        if (pipeline) gst_object_unref(pipeline);
        return nullptr;
    }

    g_object_set(G_OBJECT(source), "location", filePath, nullptr);

    /* Configure appsink to be pull-based and non-blocking */
    g_object_set(G_OBJECT(appsink), "emit-signals", FALSE, "sync", FALSE, nullptr);

    gst_bin_add_many(GST_BIN(pipeline), source, decode, convert, chromahold, convert2, appsink, NULL);

    if (!gst_element_link(source, decode)) {
        g_printerr("Failed to link source -> decodebin\n");
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (!gst_element_link(convert, chromahold)) {
        g_printerr("Failed to link videoconvert -> chromahold\n");
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (!gst_element_link(chromahold, convert2)) {
        g_printerr("Failed to link chromahold -> videoconvert2\n");
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (!gst_element_link(convert2, appsink)) {
        g_printerr("Failed to link videoconvert2 -> appsink\n");
        gst_object_unref(pipeline);
        return nullptr;
    }

    /* decodebin will create pads dynamically; connect to pad-added to link to convert */
    g_signal_connect(decode, "pad-added", G_CALLBACK(onPadAdded), convert);

    GstStateChangeReturn sret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return nullptr;
    }

    if (outAppsink) {
        *outAppsink = appsink;
    }

    return pipeline;
}

bool pollBusForMessages(GstBus* bus) {
    // Poll the bus with a short timeout to avoid blocking the appsink loop.
    GstMessage* msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
                                                  static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (msg != nullptr) {
        switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError* err = nullptr;
            gchar* debugInfo = nullptr;
            gst_message_parse_error(msg, &err, &debugInfo);
            g_printerr("Error received from element %s: %s\n",
                       GST_OBJECT_NAME(msg->src), err->message);
            g_printerr("Debugging information: %s\n",
                       debugInfo ? debugInfo : "none");
            g_clear_error(&err);
            g_free(debugInfo);
            gst_message_unref(msg);
            return true; // signal caller to break
        }
        case GST_MESSAGE_EOS:
            g_print("End-Of-Stream reached.\n");
            gst_message_unref(msg);
            return true; // signal caller to break
        default:
            /* We only asked for ERROR or EOS, but handle defensively. */
            gst_message_unref(msg);
            return false;
        }
    }
    return false; // no relevant message
}
