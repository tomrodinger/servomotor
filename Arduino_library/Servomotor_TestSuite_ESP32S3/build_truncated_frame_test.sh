#!/bin/bash
# Build the standalone transport-layer receive-path test. No hardware needed;
# it drives Communication directly against a scripted pseudo-terminal peer.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -I. -I.. \
    truncated_frame_test.cpp \
    ../Communication.cpp \
    -o truncated_frame_test
echo "Built ./truncated_frame_test"
echo "Run with: python3 run_truncated_frame_test.py"
