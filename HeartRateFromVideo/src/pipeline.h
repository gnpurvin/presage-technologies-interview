#pragma once

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

// Callback for decodebin pad-added; links the dynamic pad to the provided
// sink element.
void onPadAdded(GstElement* src, GstPad* pad, gpointer data);

// Build the pipeline and return a newly created GstElement* pipeline. The
// caller takes ownership and is responsible for setting state and unreffing.
// Returns nullptr on failure.
GstElement* buildPipeline(const char* filePath, GstElement** outAppsink);

// Poll the GstBus for ERROR or EOS messages. If an ERROR is received the
// function will print diagnostics and return true indicating the main loop
// should break. If EOS is received the function prints a message and also
// returns true. Otherwise returns false meaning the caller should continue.
bool pollBusForMessages(GstBus* bus);
