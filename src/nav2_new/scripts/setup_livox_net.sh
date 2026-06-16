#!/usr/bin/env bash
# =============================================================================
# setup_livox_net.sh — Configure the Jetson's ethernet port for the Livox Mid-360
# =============================================================================
# The Mid-360 default IP is 192.168.1.1xx (where xx is set by dials on the
# sensor). The host must be on the same /24 subnet. Default host IP: 192.168.1.5.
#
# Run ONCE after booting the Jetson (or add to systemd unit for auto-setup).
# Run INSIDE the container if running with --net=host (the container shares
# the host's network stack).
# =============================================================================
set -e

# ── User-editable ────────────────────────────────────────────────────────────
IFACE="${IFACE:-eno1}"            # change to the actual interface (check with `ip link`)
HOST_IP="${HOST_IP:-192.168.1.5}"
LIDAR_IP="${LIDAR_IP:-192.168.1.3}"  # default Mid-360 — match your dials

echo "─────────────────────────────────────────────────────────"
echo "  Configuring ${IFACE} for Livox Mid-360"
echo "  Host IP:   ${HOST_IP}/24"
echo "  LiDAR IP:  ${LIDAR_IP}"
echo "─────────────────────────────────────────────────────────"

# Bring the interface up
sudo ip link set "${IFACE}" up

# Flush existing IP addresses on that interface
sudo ip addr flush dev "${IFACE}"

# Assign static IP
sudo ip addr add "${HOST_IP}/24" dev "${IFACE}"

echo "✓ ${IFACE} configured with ${HOST_IP}/24"
echo ""
echo "Testing connectivity to LiDAR at ${LIDAR_IP}..."
if ping -c 2 -W 2 "${LIDAR_IP}" > /dev/null 2>&1; then
    echo "✓ LiDAR is reachable."
else
    echo "✗ Cannot ping ${LIDAR_IP}."
    echo "  Troubleshoot:"
    echo "  - Is the LiDAR powered on and the ethernet cable connected?"
    echo "  - Check the Mid-360's IP dials (on the sensor body)"
    echo "  - Verify with: ip addr show ${IFACE}"
    exit 1
fi
