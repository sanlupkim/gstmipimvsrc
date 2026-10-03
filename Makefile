# mipimvsrc - a GStreamer source for MIPI machine vision cameras
#
# Copyright (C) 2026 Lukasz Sanocki
#
# This program is free software: you can redistribute it and/or modify it under the terms of the GNU
# General Public License as published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version. See the LICENSE file for the full text.

# Build the mipimvsrc GStreamer plugin.
#
#   make            build
#   make install    copy into the user plugin directory (no root needed)
#   make inspect    show the element and its properties
#   make clean

PLUGIN   := libgstmipimvsrc.so
SRC      := gstmipimvsrc.cpp mvcam.cpp

# -O3 is not optional here: the unpacking loop measured 3.2x slower without it, which is the
# difference between fitting in the frame budget and dropping frames.
CXXFLAGS ?= -O3 -fPIC -Wall -Wextra
LDFLAGS  ?= -shared

PKGS     := gstreamer-1.0 gstreamer-base-1.0 opencv4
# Deferred, so a missing package produces the message below rather than noise while make parses this
PKGFLAGS  = $(shell pkg-config --cflags $(PKGS) 2>/dev/null)
PKGLIBS   = $(shell pkg-config --libs $(PKGS) 2>/dev/null)

# Where GStreamer looks for plugins without needing root
PLUGINDIR ?= $(HOME)/.local/share/gstreamer-1.0/plugins

.PHONY: all install inspect clean check-deps

all: check-deps $(PLUGIN)

check-deps:
	@pkg-config --exists $(PKGS) 2>/dev/null || { \
		echo "Missing development packages. On Debian/Raspberry Pi OS:"; \
		echo "  sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libopencv-dev"; \
		exit 1; }

# $(SRC), not $<: $< is only the first prerequisite, so mvcam.cpp was silently left out and the link
# failed on every MvCam symbol.
$(PLUGIN): $(SRC) mvcam.h
	$(CXX) $(CXXFLAGS) $(PKGFLAGS) $(LDFLAGS) -o $@ $(SRC) $(PKGLIBS)

install: $(PLUGIN)
	mkdir -p $(PLUGINDIR)
	cp $(PLUGIN) $(PLUGINDIR)/
	@# The registry caches the old plugin by path and mtime, so a stale one can survive a rebuild
	rm -f $(HOME)/.cache/gstreamer-1.0/registry.*.bin
	@echo "Installed to $(PLUGINDIR)"

inspect: install
	gst-inspect-1.0 mipimvsrc

clean:
	rm -f $(PLUGIN)
