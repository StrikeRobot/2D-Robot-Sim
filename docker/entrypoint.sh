#!/bin/bash
set -e

# ── 1. Virtual framebuffer (1280×800 24-bit) ──────────────────────────────
Xvfb :99 -screen 0 1280x800x24 -ac +extension GLX +render -noreset &
sleep 0.4

# ── 2. VNC server — password-free, localhost only ─────────────────────────
x11vnc -display :99 -nopw -listen localhost \
       -xkb -forever -shared -bg -quiet 2>/dev/null

# ── 3. noVNC web proxy ────────────────────────────────────────────────────
# websockify is installed by the 'novnc' apt package (python3-websockify dep)
websockify --web /usr/share/novnc/ 11100 localhost:5900 &

echo ""
echo "┌──────────────────────────────────────────────────────────┐"
echo "│  sr-sim is starting                                      │"
echo "│                                                          │"
echo "│  Open http://localhost:11100/vnc.html in your browser    │"
echo "│                                                          │"
echo "│  Controls: left-click=obstacle  right-click=goal  R=reset│"
echo "└──────────────────────────────────────────────────────────┘"
echo ""
sleep 0.4

exec python examples/01_single_robot_apf.py
