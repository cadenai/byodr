#!/usr/bin/env bash

G_USER=${CAMERA1_USER:=user2}
G_PASS=${CAMERA1_PASS:=HelloUser2}
G_SERVER=${CAMERA1_IP:=192.168.1.64}
G_PORT=${CAMERA1_PORT:=554}
G_PATH=${CAMERA1_PATH:=/Streaming/Channels/101}
B_PORT=${CAMERA1_SERVICE_PORT:=5101}

gst-launch-1.0 \
  rtspsrc location=rtsp://${G_USER}:${G_PASS}@${G_SERVER}:${G_PORT}${G_PATH} \
  latency=0 drop-on-latency=true \
  ! queue ! rtph264depay ! h264parse \
  ! video/x-h264,stream-format="byte-stream" \
  ! queue ! tcpserversink host=0.0.0.0 port=${B_PORT} sync=false async=false
