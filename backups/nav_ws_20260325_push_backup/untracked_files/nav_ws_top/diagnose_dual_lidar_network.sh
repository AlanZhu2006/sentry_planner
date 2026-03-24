#!/bin/bash
# 双雷达网络完整排查（重点：114 不可达时）
# 用法: ./diagnose_dual_lidar_network.sh

set -e
echo "=========================================="
echo "双雷达网络完整诊断"
echo "=========================================="

USB_IF="enx00e04c2536b0"
BUILTIN_IF="enp114s0"
HOST_182="192.168.1.2"
HOST_114="192.168.1.3"
LIDAR_182="192.168.1.182"
LIDAR_114="192.168.1.114"

echo ""
echo ">>> 1. 物理层：雷达 114 硬件检查"
echo "  [ ] 雷达 114 已上电（电源指示灯常亮/闪烁）"
echo "  [ ] 网线两端插紧：主机 USB 转以太网口 <-> 雷达 114 网口"
echo "  [ ] 若用 USB 转以太网适配器，确认适配器指示灯有 link"
echo "  [ ] 尝试：换一根网线、换一个 USB 口"
echo ""

echo ">>> 2. USB 设备与网卡识别"
echo "  查找 USB 转以太网 (enx 开头):"
ip link show 2>/dev/null | grep -E "enx|state" || true
echo ""
echo "  lsusb (USB 设备):"
lsusb | grep -iE "ethernet|rndis|asix|realtek|ch341|0b95|0e04" || echo "  (无常见 USB 网卡芯片)"
lsusb
echo ""
echo "  若 USB 网口不存在，尝试加载驱动:"
echo "    sudo modprobe cdc_ether"
echo "    sudo modprobe rndis_host"
echo "    sudo modprobe asix"
echo ""

echo ">>> 3. 网口状态与 IP 配置"
echo "  $BUILTIN_IF (内置，接雷达 182):"
ip -4 addr show $BUILTIN_IF 2>/dev/null || echo "    未找到或未配置"
ip link show $BUILTIN_IF 2>/dev/null | grep -E "state|LOWER_UP" || true
echo ""
echo "  $USB_IF (USB，接雷达 114):"
ip -4 addr show $USB_IF 2>/dev/null || echo "    未找到或未配置"
ip link show $USB_IF 2>/dev/null | grep -E "state|LOWER_UP" || true
echo ""
echo "  若 USB 网口未配置，执行:"
echo "    sudo ip link set $USB_IF up"
echo "    sudo ip addr add $HOST_114/24 dev $USB_IF"
echo ""

echo ">>> 4. 链路状态 (LOWER_UP 表示有物理连接)"
for iface in $BUILTIN_IF $USB_IF; do
  if ip link show $iface &>/dev/null; then
    st=$(ip link show $iface | grep -o "state [A-Z]*" || true)
    up=$(ip link show $iface | grep -o "LOWER_UP" || true)
    echo "  $iface: $st ${up:-NO_LINK}"
  fi
done
echo ""

echo ">>> 5. 路由表"
ip route | grep -E "192.168.1|default" || echo "  无相关路由"
echo "  双雷达需确保:"
echo "    $LIDAR_182/32 -> $BUILTIN_IF"
echo "    $LIDAR_114/32 -> $USB_IF"
echo "  若缺失，执行:"
echo "    sudo ip route add $LIDAR_114/32 dev $USB_IF"
echo "    sudo ip route add $LIDAR_182/32 dev $BUILTIN_IF"
echo ""

echo ">>> 6. Ping 测试"
echo "  雷达 182:"
ping -c 2 -W 2 -I $HOST_182 $LIDAR_182 2>&1 | tail -4
echo ""
echo "  雷达 114:"
ping -c 2 -W 2 -I $HOST_114 $LIDAR_114 2>&1 | tail -4
echo ""

echo ">>> 7. ARP 扫描 (发现 USB 网口下实际设备)"
echo "  若已安装 arp-scan，扫描 $USB_IF 所在网段:"
if command -v arp-scan &>/dev/null; then
  echo "  执行: sudo arp-scan --interface=$USB_IF --localnet"
  sudo arp-scan --interface=$USB_IF --localnet 2>/dev/null || echo "  (需 root 或 arp-scan 未安装)"
else
  echo "  安装: sudo apt install arp-scan"
  echo "  然后: sudo arp-scan --interface=$USB_IF --localnet"
fi
echo ""

echo ">>> 8. 重要：Livox MID360 IP 规则"
echo "  MID360 出厂 IP = 192.168.1.XX，XX = 序列号最后两位"
echo "  例如：序列号 xxx14 -> 192.168.1.114"
echo "  若你的雷达 114 序列号不是 ...14，IP 可能不是 192.168.1.114"
echo "  用 Livox Viewer2 或 arp-scan 确认实际 IP"
echo ""

echo ">>> 9. 全网段扫描 (可选，需 nmap)"
if command -v nmap &>/dev/null; then
  echo "  扫描 192.168.1.0/24 存活主机:"
  nmap -sn 192.168.1.0/24 2>/dev/null | grep -E "Nmap scan|192.168" || echo "  (可能需 sudo)"
else
  echo "  安装: sudo apt install nmap"
  echo "  然后: sudo nmap -sn 192.168.1.0/24"
fi
echo ""

echo ">>> 10. 排查结论与建议"
echo "  若 182 通、114 不通，按顺序检查:"
echo "  1. 雷达 114 上电、网线插紧、换线/换 USB 口"
echo "  2. USB 网口 LOWER_UP，且已配置 $HOST_114/24"
echo "  3. 路由: $LIDAR_114/32 -> $USB_IF"
echo "  4. 用 arp-scan 确认 114 网段下是否有设备，以及实际 IP"
echo "  5. 确认雷达 114 序列号末两位是否为 14（否则 IP 不是 .114）"
echo "  6. 若 114 和 182 接在同一交换机，可能需不同网段或 VLAN"
echo ""
