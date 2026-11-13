#!/usr/bin/env bash
# Run INSIDE the running bridge container (scripts/verify_isolation.sh does that).
# Fails if any TCP listener other than the RR port is reachable from outside the
# container, or if any UDP socket (DDS) is bound to a non-loopback address.
set -uo pipefail
PORT="${RR_NAV2_PORT:-62354}"
fail=0

echo "== environment"
for v in ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DOMAIN_ID RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE; do
    echo "  $v=${!v:-<unset>}"
done
[[ "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" == "LOCALHOST" ]] || { echo "FAIL: discovery range not LOCALHOST"; fail=1; }

is_loopback() { [[ "$1" == 127.* || "$1" == "[::1]" || "$1" == "::1" ]]; }

echo "== TCP listeners"
while read -r local; do
    addr="${local%:*}"; p="${local##*:}"
    if is_loopback "$addr"; then
        echo "  ok   $local (loopback)"
    elif [[ "$p" == "$PORT" ]]; then
        echo "  ok   $local (RR port)"
    else
        echo "  FAIL $local is reachable from outside the container"; fail=1
    fi
done < <(ss -Hlnt | awk '{print $4}')

echo "== UDP sockets"
# DDS receive ports for this domain (RTPS: 7400 + 250*domain .. +249) and the Fast DDS
# SHM watchdog ports (7000-7999) must be loopback-only. Fast DDS also opens one *send*
# socket per interface on an ephemeral port; it never reads from those, so they are
# reported but allowed.
dom="${ROS_DOMAIN_ID:-0}"; dds_lo=$((7400 + 250*dom)); dds_hi=$((dds_lo + 249))
read -r eph_lo _ < /proc/sys/net/ipv4/ip_local_port_range
count=0; send=0
while read -r local; do
    addr="${local%:*}"; p="${local##*:}"; count=$((count+1))
    is_loopback "$addr" && continue
    if (( (p >= dds_lo && p <= dds_hi) || (p >= 7000 && p <= 7999) || p < eph_lo )); then
        echo "  FAIL UDP receive port $local is bound to a non-loopback address"; fail=1
    else
        send=$((send+1))
    fi
done < <(ss -Hlnu | awk '{print $4}')
echo "  $count UDP sockets; DDS ports $dds_lo-$dds_hi and 7000-7999 loopback-only;"
echo "  $send ephemeral DDS send sockets on non-loopback (not read; allowed)"
# Multicast discovery must be off (LOCALHOST range): port dds_lo must not be open at all.
if ss -Hlnu | awk '{print $4}' | grep -q ":${dds_lo}\$"; then
    echo "  FAIL multicast discovery port $dds_lo is open"; fail=1
fi

if [[ $fail -eq 0 ]]; then echo "ISOLATION OK"; else echo "ISOLATION FAILED"; fi
exit $fail
