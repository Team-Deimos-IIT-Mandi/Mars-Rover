#!/bin/bash

# Configure CAN interface (adjust parameters based on your MCP2515 setup)
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Verify
ip -details link show can0
