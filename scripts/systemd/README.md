# systemd — On-Robot Systemd Units

This directory contains persistent **systemd service units** deployed on robot companion computers (e.g., Jetson Orin NX).

## File Inventory

| Unit File | Companion Script | Purpose |
|---|---|---|
| `sw01-sdk-port-redirect.service` | `sw01-sdk-port-redirect.sh` (in this directory) | **SW01 SDK reply port redirect**: SW01 firmware >= 1.1.7 sends UDP replies to port 3601 while ASTRALL SDK 1.0.7 binds to UDP port 3600. Redirects `UDP 3601 -> 3600` via `iptables` NAT `REDIRECT` at boot. |
| `web-console.service` | — (runs `python3 -m web_console.run`) | **ONS2 web console** on port 18090 (map, missions, e-stop, teleop, camera, intercom). Replaces the retired `sw01-remote-monitor.service`. |

## Installation (on the SW01 host)

```bash
REPO=<path_to_this_repository>

sudo install -m 755 "$REPO/scripts/systemd/sw01-sdk-port-redirect.sh" /usr/local/sbin/sw01-sdk-port-redirect.sh
sudo install -m 644 "$REPO/scripts/systemd/sw01-sdk-port-redirect.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now sw01-sdk-port-redirect.service

# Optional: restrict to a single network interface (default: all interfaces)
echo 'SW01_SDK_IFACE=eno1' | sudo tee /etc/default/sw01-sdk-port-redirect && sudo systemctl restart sw01-sdk-port-redirect.service
```

Verification:

```bash
systemctl is-active sw01-sdk-port-redirect.service          # active
sudo /usr/local/sbin/sw01-sdk-port-redirect.sh status        # should show 1 rule redirecting dpt:3601 -> 3600
```

> Note: Disable this unit (`sudo systemctl disable --now sw01-sdk-port-redirect.service`) if upgrading to an ASTRALL SDK version (v1.0.8+) that natively listens on port 3601.

## Web console autostart (SW01 host)

The old `sw01-remote-monitor.service` (port 18088, `sw01_remote_monitor.py`) is retired: stop, disable and mask it, then install the web console unit.

```bash
REPO=/home/mt/base_inspection

# 1. retire the old monitor (mask keeps it from being started by anything else)
sudo systemctl disable --now sw01-remote-monitor.service
sudo systemctl mask sw01-remote-monitor.service

# 2. stop any manually started web console that still holds port 18090
#    (pid file written when it was started by hand; skip if none)
[ -f /home/mt/web_console.pid ] && kill "$(cat /home/mt/web_console.pid)" 2>/dev/null; rm -f /home/mt/web_console.pid

# 3. install and start the web console at boot
sudo install -m 644 "$REPO/scripts/systemd/web-console.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now web-console.service
```

Verification:

```bash
systemctl is-active web-console.service      # active
systemctl is-enabled web-console.service     # enabled
curl -s http://127.0.0.1:18090/api/state | head -c 120; echo   # api_version 2 is in the JSON
journalctl -u web-console.service -n 20 --no-pager
```

Rollback to the old monitor: `sudo systemctl unmask sw01-remote-monitor.service && sudo systemctl enable --now sw01-remote-monitor.service`.
