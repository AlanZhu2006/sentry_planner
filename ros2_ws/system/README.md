# Jetson host configuration

These are deliberate host-level settings that are not applied automatically.
They reproduce the current Jetson without committing a VNC password, Wi-Fi
credential, USB device identity, or other secret.

## MID-360 Ethernet

The tested hardware uses `enP8p1s0`, Jetson address `192.168.1.50/24`, and
MID-360 address `192.168.1.182`. The LiDAR link must not replace Wi-Fi as the
default route:

```bash
sudo nmcli connection modify "Wired connection 1" \
  connection.interface-name enP8p1s0 \
  connection.autoconnect yes \
  ipv4.method manual \
  ipv4.addresses 192.168.1.50/24 \
  ipv4.gateway "" \
  ipv4.never-default yes
sudo nmcli connection up "Wired connection 1"
```

Verify the interface and then update
`mid360_nav_bringup/config/mid360_config.json` if either address differs:

```bash
ip -br address show enP8p1s0
ping 192.168.1.182
```

## Persistent TigerVNC display `:1`

Install TigerVNC, create the password interactively, and map display `:1` to
the login user. Never commit `~/.vnc/passwd`.

```bash
sudo apt install tigervnc-standalone-server tigervnc-common
vncpasswd
sudoedit /etc/tigervnc/vncserver.users
```

The mapping file needs one line (replace the user when appropriate):

```text
:1=nyu
```

Copy `tigervnc.conf.example` to `~/.vnc/tigervnc.conf`, then enable the distro
service:

```bash
mkdir -p ~/.vnc
cp ros2_ws/system/tigervnc.conf.example ~/.vnc/tigervnc.conf
sudo systemctl enable --now tigervncserver@:1.service
systemctl status tigervncserver@:1.service
```

The current configuration listens on the LAN at port 5901 so RealVNC Viewer
can connect to `JETSON_IP:5901`. Use this only on a trusted network and restrict
5901 with the firewall. For an untrusted network, set `$localhost = "yes";`
and connect through an SSH tunnel instead:

```bash
ssh -C -L 5901:localhost:5901 USER@JETSON_IP
```

Once the systemd unit owns display `:1`, do not run `vncserver -kill :1` and
`vncserver :1 ...` on every boot. The mapping/navigation launchers detect SSH
and send RViz to the existing `:1` display. Override with `NAV_RVIZ_DISPLAY`
and `NAV_RVIZ_XAUTHORITY` only when intentionally using a different display.
