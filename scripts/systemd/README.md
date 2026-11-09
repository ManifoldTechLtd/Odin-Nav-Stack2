# systemd — On-Robot Systemd Units

This directory contains persistent **systemd service units** deployed on robot companion computers (e.g., Jetson Orin NX).

## File Inventory

| Unit File | Companion Script | Purpose |
|---|---|---|
| `sw01-sdk-port-redirect.service` | `sw01-sdk-port-redirect.sh` (in this directory) | **SW01 SDK reply port redirect**: SW01 firmware >= 1.1.7 sends UDP replies to port 3601 while ASTRALL SDK 1.0.7 binds to UDP port 3600. Redirects `UDP 3601 -> 3600` via `iptables` NAT `REDIRECT` at boot. |

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
