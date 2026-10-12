#!/usr/bin/env bash
#
# Quick SocketCAN setup helper for MASC chassis CAN devices.
#
# This script is intended for temporary field setup and diagnostics. Long-term
# boot-time setup should still use systemd-networkd or a dedicated slcand service.

set -euo pipefail

DEFAULT_IFACE="can0"
DEFAULT_BITRATE="500000"
DEFAULT_RESTART_MS="100"
DEFAULT_TEST_TIMEOUT="5"

MODE="${1:-help}"
if [[ $# -gt 0 ]]; then
    shift
fi

IFACE="${DEFAULT_IFACE}"
BITRATE="${DEFAULT_BITRATE}"
DEVICE=""
RESTART_MS="${DEFAULT_RESTART_MS}"
TEST_TIMEOUT="${DEFAULT_TEST_TIMEOUT}"
SKIP_TEST=0
EXPLICIT_IFACE=0

usage() {
    cat <<'EOF'
Usage:
  setup_can.sh auto [options]
  setup_can.sh socketcan [options]
  setup_can.sh slcan --device DEVICE [options]
  setup_can.sh status [--iface IFACE]
  setup_can.sh test [--iface IFACE] [--test-timeout SEC]
  setup_can.sh down [--iface IFACE]
  setup_can.sh list

Modes:
  auto       Prefer an existing SocketCAN interface. If none exists, use one
             unique SLCAN serial candidate. Multiple serial candidates require
             --device.
  socketcan  Configure an existing native SocketCAN interface, such as gs_usb,
             Candlelight, mcp251x, or a board CAN controller.
  slcan      Start slcand from a serial device and create a SocketCAN interface.
  status     Show current CAN interface details and statistics.
  test       Listen for MASC system_version_feedback (0x205) for a few seconds.
  down       Bring the interface down and stop a matching slcand process if any.
  list       List CAN interfaces and serial candidates.

Options:
  -i, --iface IFACE          SocketCAN interface name. Default: can0
  -b, --bitrate BITRATE      CAN bitrate. Default: 500000
  -d, --device DEVICE        Serial device for SLCAN, for example /dev/ttyACM0
                             or /dev/serial/by-id/...
      --restart-ms MS        SocketCAN bus-off auto restart time. Default: 100
      --test-timeout SEC     Timeout for test mode. Default: 5
      --skip-test            Do not run the 0x205 candump check after setup.
  -h, --help                 Show this help.

Examples:
  sudo ./scripts/setup_can.sh auto
  sudo ./scripts/setup_can.sh socketcan --iface can0 --bitrate 500000
  sudo ./scripts/setup_can.sh slcan --device /dev/ttyACM0 --iface can0
  ./scripts/setup_can.sh status --iface can0
  ./scripts/setup_can.sh test --iface can0
EOF
}

log() {
    printf '[setup_can] %s\n' "$*"
}

warn() {
    printf '[setup_can] warning: %s\n' "$*" >&2
}

die() {
    printf '[setup_can] error: %s\n' "$*" >&2
    exit 1
}

require_root() {
    if [[ "${EUID}" -ne 0 ]]; then
        die "mode '${MODE}' requires root. Please run with sudo."
    fi
}

require_cmd() {
    local cmd
    for cmd in "$@"; do
        command -v "${cmd}" >/dev/null 2>&1 || die "command '${cmd}' not found"
    done
}

run() {
    log "+ $*"
    "$@"
}

try_modprobe() {
    local module="$1"
    if modprobe "${module}" >/dev/null 2>&1; then
        log "loaded kernel module: ${module}"
    else
        warn "could not load kernel module '${module}', continuing"
    fi
}

list_can_ifaces() {
    ip -o link show type can 2>/dev/null | awk -F': ' '{print $2}' | cut -d: -f1 | cut -d@ -f1
}

iface_exists() {
    ip link show "$1" >/dev/null 2>&1
}

iface_is_can() {
    local iface="$1"
    list_can_ifaces | grep -Fxq "${iface}"
}

collect_serial_candidates() {
    local path

    if [[ -d /dev/serial/by-id ]]; then
        while IFS= read -r path; do
            [[ -n "${path}" ]] && printf '%s\n' "${path}"
        done < <(find /dev/serial/by-id -maxdepth 1 -type l 2>/dev/null | sort)
    fi

    if ! compgen -G "/dev/serial/by-id/*" >/dev/null 2>&1; then
        for path in /dev/ttyACM* /dev/ttyUSB*; do
            [[ -e "${path}" ]] && printf '%s\n' "${path}"
        done
    fi
}

print_list() {
    require_cmd ip

    log "CAN interfaces:"
    if ! list_can_ifaces | sed 's/^/  /'; then
        true
    fi
    if [[ -z "$(list_can_ifaces || true)" ]]; then
        printf '  (none)\n'
    fi

    log "SLCAN serial candidates:"
    local candidates
    candidates="$(collect_serial_candidates || true)"
    if [[ -n "${candidates}" ]]; then
        printf '%s\n' "${candidates}" | sed 's/^/  /'
    else
        printf '  (none)\n'
    fi
}

slcan_speed_arg() {
    case "$1" in
        10000) echo "-s0" ;;
        20000) echo "-s1" ;;
        50000) echo "-s2" ;;
        100000) echo "-s3" ;;
        125000) echo "-s4" ;;
        250000) echo "-s5" ;;
        500000) echo "-s6" ;;
        800000) echo "-s7" ;;
        1000000) echo "-s8" ;;
        *) die "unsupported SLCAN bitrate '$1'. Supported: 10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000" ;;
    esac
}

stop_slcand_for_iface() {
    local iface="$1"
    local pattern="slcand.*[[:space:]]${iface}([[:space:]]|$)"

    if pgrep -af "${pattern}" >/dev/null 2>&1; then
        log "stopping slcand process for ${iface}"
        pgrep -af "${pattern}" | sed 's/^/[setup_can]   /'
        pkill -f "${pattern}" || true
        sleep 0.3
    fi
}

wait_for_iface() {
    local iface="$1"
    local i
    for i in $(seq 1 30); do
        if iface_exists "${iface}"; then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

show_status() {
    require_cmd ip
    run ip -details -statistics link show "${IFACE}"
}

test_can() {
    require_cmd candump timeout

    local tmp
    local err
    tmp="$(mktemp)"
    err="$(mktemp)"

    log "listening for system_version_feedback (0x205) on ${IFACE} for ${TEST_TIMEOUT}s"
    timeout "${TEST_TIMEOUT}" candump -tz "${IFACE},205:7FF" >"${tmp}" 2>"${err}" || true

    if [[ -s "${tmp}" ]]; then
        cat "${tmp}"
        rm -f "${tmp}" "${err}"
        log "received system_version_feedback (0x205)"
        return 0
    fi

    if [[ -s "${err}" ]]; then
        cat "${err}" >&2
        rm -f "${tmp}" "${err}"
        warn "candump failed on ${IFACE}"
        return 1
    fi

    rm -f "${tmp}" "${err}"
    warn "no 0x205 frame received on ${IFACE} within ${TEST_TIMEOUT}s"
    warn "check chassis power, CANH/CANL wiring, termination, bitrate, and interface name"
    return 1
}

run_optional_test() {
    if [[ "${SKIP_TEST}" -ne 0 ]]; then
        return
    fi

    if ! command -v candump >/dev/null 2>&1; then
        warn "candump not found; install can-utils to enable 0x205 validation"
        warn "sudo apt install -y can-utils"
        return
    fi
    if ! command -v timeout >/dev/null 2>&1; then
        warn "timeout command not found; skipping 0x205 validation"
        return
    fi

    test_can || true
}

configure_socketcan() {
    require_root
    require_cmd ip modprobe

    try_modprobe can
    try_modprobe can_raw
    try_modprobe gs_usb

    iface_exists "${IFACE}" || die "interface '${IFACE}' does not exist"
    iface_is_can "${IFACE}" || die "interface '${IFACE}' exists but is not a CAN interface"

    log "configuring SocketCAN interface ${IFACE} at ${BITRATE} bit/s"
    run ip link set "${IFACE}" down || true
    if ! ip link set "${IFACE}" type can bitrate "${BITRATE}" restart-ms "${RESTART_MS}"; then
        warn "failed to set bitrate with restart-ms on ${IFACE}; retrying without restart-ms"
        run ip link set "${IFACE}" type can bitrate "${BITRATE}"
    else
        log "+ ip link set ${IFACE} type can bitrate ${BITRATE} restart-ms ${RESTART_MS}"
    fi
    run ip link set "${IFACE}" up
    show_status

    run_optional_test
}

configure_slcan() {
    require_root
    require_cmd ip modprobe slcand

    [[ -n "${DEVICE}" ]] || die "slcan mode requires --device"
    [[ -e "${DEVICE}" ]] || die "serial device '${DEVICE}' does not exist"

    try_modprobe can
    try_modprobe can_raw
    try_modprobe slcan

    stop_slcand_for_iface "${IFACE}"
    if iface_exists "${IFACE}"; then
        run ip link set "${IFACE}" down || true
    fi
    if iface_exists "${IFACE}"; then
        die "interface '${IFACE}' already exists; choose another --iface or stop the process that owns it"
    fi

    local speed_arg
    speed_arg="$(slcan_speed_arg "${BITRATE}")"

    log "creating SLCAN interface ${IFACE} from ${DEVICE} at ${BITRATE} bit/s"
    run slcand -o -c "${speed_arg}" "${DEVICE}" "${IFACE}"
    wait_for_iface "${IFACE}" || die "slcand did not create interface '${IFACE}'"
    run ip link set "${IFACE}" up
    show_status

    run_optional_test
}

choose_auto_mode() {
    require_root
    require_cmd ip

    local can_ifaces=()
    mapfile -t can_ifaces < <(list_can_ifaces || true)

    if iface_exists "${IFACE}"; then
        if iface_is_can "${IFACE}"; then
            log "auto: using existing CAN interface ${IFACE}"
            configure_socketcan
            return
        fi
        die "auto: requested interface '${IFACE}' exists but is not a CAN interface"
    fi

    if [[ "${EXPLICIT_IFACE}" -eq 0 ]]; then
        if [[ "${#can_ifaces[@]}" -eq 1 ]]; then
            IFACE="${can_ifaces[0]}"
            log "auto: using only CAN interface ${IFACE}"
            configure_socketcan
            return
        fi
        if [[ "${#can_ifaces[@]}" -gt 1 ]]; then
            if printf '%s\n' "${can_ifaces[@]}" | grep -Fxq "${DEFAULT_IFACE}"; then
                IFACE="${DEFAULT_IFACE}"
                log "auto: multiple CAN interfaces found, using ${IFACE}"
                configure_socketcan
                return
            fi
            print_list
            die "auto: multiple CAN interfaces found; specify --iface"
        fi
    fi

    if [[ -z "${DEVICE}" ]]; then
        local serial_candidates=()
        mapfile -t serial_candidates < <(collect_serial_candidates || true)
        if [[ "${#serial_candidates[@]}" -eq 1 ]]; then
            DEVICE="${serial_candidates[0]}"
            log "auto: using only SLCAN serial candidate ${DEVICE}"
        elif [[ "${#serial_candidates[@]}" -gt 1 ]]; then
            print_list
            die "auto: multiple serial candidates found; specify --device"
        else
            print_list
            die "auto: no CAN interface or SLCAN serial candidate found"
        fi
    fi

    configure_slcan
}

bring_down() {
    require_root
    require_cmd ip

    stop_slcand_for_iface "${IFACE}"
    if iface_exists "${IFACE}"; then
        run ip link set "${IFACE}" down || true
    else
        warn "interface '${IFACE}' does not exist"
    fi
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -i|--iface)
                [[ $# -ge 2 ]] || die "$1 requires a value"
                IFACE="$2"
                EXPLICIT_IFACE=1
                shift 2
                ;;
            -b|--bitrate)
                [[ $# -ge 2 ]] || die "$1 requires a value"
                BITRATE="$2"
                shift 2
                ;;
            -d|--device)
                [[ $# -ge 2 ]] || die "$1 requires a value"
                DEVICE="$2"
                shift 2
                ;;
            --restart-ms)
                [[ $# -ge 2 ]] || die "$1 requires a value"
                RESTART_MS="$2"
                shift 2
                ;;
            --test-timeout)
                [[ $# -ge 2 ]] || die "$1 requires a value"
                TEST_TIMEOUT="$2"
                shift 2
                ;;
            --skip-test|--no-test)
                SKIP_TEST=1
                shift
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                die "unknown option '$1'"
                ;;
        esac
    done
}

parse_args "$@"

case "${MODE}" in
    auto)
        choose_auto_mode
        ;;
    socketcan)
        configure_socketcan
        ;;
    slcan)
        configure_slcan
        ;;
    status)
        show_status
        ;;
    test)
        test_can
        ;;
    down)
        bring_down
        ;;
    list)
        print_list
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        die "unknown mode '${MODE}'"
        ;;
esac
