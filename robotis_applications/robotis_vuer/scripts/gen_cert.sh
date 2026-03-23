#!/bin/bash

set -e

detect_host_ip() {
  local detected_ip=""

  if command -v ip >/dev/null 2>&1; then
    detected_ip="$(ip route get 1.1.1.1 2>/dev/null | awk '/src/ {print $7; exit}')"
  fi

  if [ -z "${detected_ip}" ] && command -v hostname >/dev/null 2>&1; then
    detected_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi

  printf '%s\n' "${detected_ip}"
}

HOST_IP="${1:-$(detect_host_ip)}"
VR_FOLDER="/root/ros2_ws/src/robotis_motion_controller/robotis_applications/robotis_vuer/robotis_vuer"

if [ -z "${HOST_IP}" ]; then
  echo "Error: failed to detect host IP automatically." >&2
  echo "Usage: $0 [HOST_IP]" >&2
  echo "Example: $0 192.168.0.10" >&2
  exit 1
fi

echo "1. Installing mkcert root CA..."
mkcert -install

echo "2. Generating certificate (cert.pem, key.pem) for ${HOST_IP}"
mkcert -cert-file cert.pem -key-file key.pem "$HOST_IP" localhost 127.0.0.1

echo "3. Copying certificates to Robotis Vuer folder ($VR_FOLDER)"
mkdir -p "$VR_FOLDER"
cp cert.pem key.pem "$VR_FOLDER"

echo "Done! Certificates have been copied to $VR_FOLDER."