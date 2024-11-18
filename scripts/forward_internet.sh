#!/usr/bin/env bash
declare -r BOLD_STYLE='[1m'
declare -r CLEAR_STYLE='[0m'

print_help() {
  cat <<HELP
Fowards your internet to the LAN from WAN, giving the LAN internet access

USAGE: ./forward_internet.sh <WAN-internet-interface> <LAN-local-interface>

Use \`ip a\` to list your interfaces. The static ip of the LAN interface \
${BOLD_STYLE}must${CLEAR_STYLE} be set to ${BOLD_STYLE}10.42.43.1${CLEAR_STYLE}

Example:
  ./forward_internet.sh wlan0 eth0
HELP
}

declare -r WANIF="$1"
declare -r LANIF="$2"

if [[ -z "$LANIF" ]]; then
  print_help
  exit 1
fi

# Accept forwarding
sudo iptables -P INPUT ACCEPT
sudo iptables -P OUTPUT ACCEPT
sudo iptables -P FORWARD ACCEPT

# Forward internet
sudo sysctl net.ipv4.ip_forward=1

sudo iptables -t nat -A POSTROUTING -o "$WANIF" -j MASQUERADE
sudo iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i "$LANIF" -o "$WANIF" -j ACCEPT
