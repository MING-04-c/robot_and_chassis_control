#!/usr/bin/env bash
#
# Install and manage persistent systemd services for MASC CAN interfaces.

set -euo pipefail

DEFAULT_IFACE="can0"
DEFAULT_BITRATE="500000"
DEFAULT_RESTART_MS="100"
SERVICE_PREFIX="masc-can"
ENV_DIR="/etc/masc/can"
SYSTEMD_DIR="/etc/systemd/system"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_CAN="${SCRIPT_DIR}/setup_can.sh"

COMMAND="${1:-help}"
if [[ $# -gt 0 ]]; then
    shift
fi

INSTALL_MODE="auto"
if [[ "${COMMAND}" == "install" && $# -gt 0 ]]; then
    case "$1" in
        auto|socketcan|slcan)
            INSTALL_MODE="$1"
            shift
            ;;
    esac
fi

IFACE="${DEFAULT_IFACE}"
BITRATE="${DEFAULT_BITRATE}"
RESTART_MS="${DEFAULT_RESTART_MS}"
DEVICE=""
YES=0
DRY_RUN=0
RUN_TEST=0
EXPLICIT_IFACE=0
RESOLVED_MODE=""

usage() {
    cat <<'EOF'
Usage:
  can_service.sh install [auto|socketcan|slcan] [options]
  can_service.sh uninstall [--iface IFACE]
  can_service.sh status [--iface IFACE]
  can_service.sh restart [--iface IFACE]
  can_service.sh show [--iface IFACE]

Install options:
  -i, --iface IFACE          SocketCAN interface name. Default: can0
  -b, --bitrate BITRATE      CAN bitrate. Default: 500000
  -d, --device DEVICE        Serial device for SLCAN, preferably /dev/serial/by-id/...
      --restart-ms MS        SocketCAN bus-off auto restart time. Default: 100
      --yes                  Install without asking for confirmation
      --dry-run              Print generated files without writing to /etc
      --test                 Run setup_can.sh test after starting the service
  -h, --help                 Show this help.

Examples:
  sudo ./scripts/can_service.sh install
  sudo ./scripts/can_service.sh install --yes
  sudo ./scripts/can_service.sh install socketcan --iface can0 --bitrate 500000
  sudo ./scripts/can_service.sh install slcan --device /dev/serial/by-id/usb-xxx --iface can0
  sudo ./scripts/can_service.sh uninstall --iface can0
  ./scripts/can_service.sh show --iface can0
EOF
}

log() {
    printf '[can_service] %s\n' "$*"
}

warn() {
    printf '[can_service] warning: %s\n' "$*" >&2
}

die() {
    printf '[can_service] error: %s\n' "$*" >&2
    exit 1
}

require_root() {
    if [[ "${EUID}" -ne 0 ]]; then
        die "command '${COMMAND}' requires root. Please run with sudo."
    fi
}

require_cmd() {
    local cmd
    for cmd in "$@"; do
        command -v "${cmd}" >/dev/null 2>&1 || die "command '${cmd}' not found"
    done
}

service_name() {
    printf '%s-%s.service' "${SERVICE_PREFIX}" "${IFACE}"
}

env_file() {
    printf '%s/%s.env' "${ENV_DIR}" "${IFACE}"
}

service_file() {
    printf '%s/%s' "${SYSTEMD_DIR}" "$(service_name)"
}

shell_quote() {
    local value="$1"
    printf "'%s'" "${value//\'/\'\\\'\'}"
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

print_candidates() {
    log "CAN interfaces:"
    local can_ifaces
    can_ifaces="$(list_can_ifaces || true)"
    if [[ -n "${can_ifaces}" ]]; then
        printf '%s\n' "${can_ifaces}" | sed 's/^/  /'
    else
        printf '  (none)\n'
    fi

    log "SLCAN serial candidates:"
    local serial_candidates
    serial_candidates="$(collect_serial_candidates || true)"
    if [[ -n "${serial_candidates}" ]]; then
        printf '%s\n' "${serial_candidates}" | sed 's/^/  /'
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

warn_unstable_device_path() {
    if [[ "${RESOLVED_MODE}" == "slcan" && "${DEVICE}" != /dev/serial/by-id/* ]]; then
        warn "SLCAN device '${DEVICE}' is not a stable /dev/serial/by-id path"
        warn "USB device numbering may change after reboot or replug"
    fi
}

resolve_install_mode() {
    require_cmd ip

    case "${INSTALL_MODE}" in
        socketcan)
            RESOLVED_MODE="socketcan"
            return
            ;;
        slcan)
            [[ -n "${DEVICE}" ]] || die "install slcan requires --device"
            RESOLVED_MODE="slcan"
            return
            ;;
        auto)
            ;;
        *)
            die "unknown install mode '${INSTALL_MODE}'"
            ;;
    esac

    if [[ -n "${DEVICE}" ]]; then
        RESOLVED_MODE="slcan"
        return
    fi

    if iface_exists "${IFACE}"; then
        iface_is_can "${IFACE}" || die "interface '${IFACE}' exists but is not a CAN interface"
        RESOLVED_MODE="socketcan"
        return
    fi

    if [[ "${EXPLICIT_IFACE}" -eq 0 ]]; then
        local can_ifaces=()
        mapfile -t can_ifaces < <(list_can_ifaces || true)
        if [[ "${#can_ifaces[@]}" -eq 1 ]]; then
            IFACE="${can_ifaces[0]}"
            RESOLVED_MODE="socketcan"
            return
        fi
        if [[ "${#can_ifaces[@]}" -gt 1 ]]; then
            print_candidates
            die "multiple CAN interfaces found; specify --iface"
        fi
    fi

    local serial_candidates=()
    mapfile -t serial_candidates < <(collect_serial_candidates || true)
    if [[ "${#serial_candidates[@]}" -eq 1 ]]; then
        DEVICE="${serial_candidates[0]}"
        RESOLVED_MODE="slcan"
        return
    fi
    if [[ "${#serial_candidates[@]}" -gt 1 ]]; then
        print_candidates
        die "multiple serial candidates found; specify --device"
    fi

    if [[ "${DRY_RUN}" -eq 1 ]]; then
        warn "no CAN hardware detected; dry-run will preview default socketcan service for ${IFACE}"
        RESOLVED_MODE="socketcan"
        return
    fi

    print_candidates
    die "no CAN interface or SLCAN serial candidate found"
}

generate_env_content() {
    local slcan_speed=""
    if [[ "${RESOLVED_MODE}" == "slcan" ]]; then
        slcan_speed="$(slcan_speed_arg "${BITRATE}")"
    fi

    cat <<EOF
CAN_MODE=$(shell_quote "${RESOLVED_MODE}")
CAN_IFACE=$(shell_quote "${IFACE}")
CAN_BITRATE=$(shell_quote "${BITRATE}")
CAN_RESTART_MS=$(shell_quote "${RESTART_MS}")
CAN_DEVICE=$(shell_quote "${DEVICE}")
CAN_SLCAN_SPEED=$(shell_quote "${slcan_speed}")
EOF
}

generate_socketcan_service() {
    local env_path
    env_path="$(env_file)"
    cat <<EOF
[Unit]
Description=MASC CAN interface ${IFACE}
After=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
EnvironmentFile=${env_path}
ExecStartPre=-/usr/sbin/modprobe can
ExecStartPre=-/usr/sbin/modprobe can_raw
ExecStartPre=-/usr/sbin/modprobe gs_usb
ExecStart=/bin/bash -c 'set -a; . ${env_path}; set +a; /usr/sbin/ip link set "\${CAN_IFACE}" down || true; if ! /usr/sbin/ip link set "\${CAN_IFACE}" type can bitrate "\${CAN_BITRATE}" restart-ms "\${CAN_RESTART_MS}"; then /usr/sbin/ip link set "\${CAN_IFACE}" type can bitrate "\${CAN_BITRATE}"; fi; /usr/sbin/ip link set "\${CAN_IFACE}" up'
ExecStop=/bin/bash -c 'set -a; . ${env_path}; set +a; /usr/sbin/ip link set "\${CAN_IFACE}" down || true'

[Install]
WantedBy=multi-user.target
EOF
}

generate_slcan_service() {
    local env_path
    env_path="$(env_file)"
    cat <<EOF
[Unit]
Description=MASC SLCAN interface ${IFACE}
After=systemd-udevd.service
Wants=systemd-udevd.service

[Service]
Type=simple
EnvironmentFile=${env_path}
ExecStartPre=-/usr/sbin/modprobe can
ExecStartPre=-/usr/sbin/modprobe can_raw
ExecStartPre=-/usr/sbin/modprobe slcan
ExecStart=/bin/bash -c 'set -a; . ${env_path}; set +a; exec /usr/bin/slcand -F -o -c "\${CAN_SLCAN_SPEED}" "\${CAN_DEVICE}" "\${CAN_IFACE}"'
ExecStartPost=/bin/bash -c 'set -a; . ${env_path}; set +a; for i in {1..30}; do /usr/sbin/ip link show "\${CAN_IFACE}" >/dev/null 2>&1 && exit 0; sleep 0.2; done; exit 1'
ExecStartPost=/bin/bash -c 'set -a; . ${env_path}; set +a; /usr/sbin/ip link set "\${CAN_IFACE}" up'
ExecStopPost=-/bin/bash -c 'set -a; . ${env_path}; set +a; /usr/sbin/ip link set "\${CAN_IFACE}" down || true'
Restart=on-failure
RestartSec=2
StartLimitIntervalSec=0

[Install]
WantedBy=multi-user.target
EOF
}

generate_service_content() {
    case "${RESOLVED_MODE}" in
        socketcan) generate_socketcan_service ;;
        slcan) generate_slcan_service ;;
        *) die "cannot generate service for unresolved mode '${RESOLVED_MODE}'" ;;
    esac
}

print_install_summary() {
    log "recommended mode: ${RESOLVED_MODE}"
    log "service name: $(service_name)"
    log "config file: $(env_file)"
    log "service file: $(service_file)"
    log "interface: ${IFACE}"
    log "bitrate: ${BITRATE}"
    if [[ "${RESOLVED_MODE}" == "socketcan" ]]; then
        log "restart-ms: ${RESTART_MS}"
    else
        log "device: ${DEVICE}"
        log "slcan speed: $(slcan_speed_arg "${BITRATE}")"
    fi
}

print_generated_files() {
    printf '\n# %s\n' "$(env_file)"
    generate_env_content
    printf '\n# %s\n' "$(service_file)"
    generate_service_content
}

confirm_install() {
    if [[ "${YES}" -eq 1 ]]; then
        return
    fi
    if [[ ! -t 0 ]]; then
        die "confirmation required in non-interactive mode; use --yes or --dry-run"
    fi

    local answer
    read -r -p "Install and enable $(service_name)? [y/N] " answer
    case "${answer}" in
        y|Y|yes|YES) ;;
        *) die "installation cancelled" ;;
    esac
}

install_service() {
    if [[ "${DRY_RUN}" -eq 0 ]]; then
        require_root
    fi

    resolve_install_mode
    warn_unstable_device_path
    print_install_summary

    if [[ "${DRY_RUN}" -eq 1 ]]; then
        print_generated_files
        return
    fi

    require_cmd systemctl
    confirm_install

    mkdir -p "${ENV_DIR}"
    generate_env_content >"$(env_file)"
    generate_service_content >"$(service_file)"

    log "+ systemctl daemon-reload"
    systemctl daemon-reload
    log "+ systemctl enable --now $(service_name)"
    systemctl enable --now "$(service_name)"
    log "+ systemctl status $(service_name) --no-pager"
    systemctl status "$(service_name)" --no-pager || true

    if [[ "${RUN_TEST}" -eq 1 ]]; then
        if [[ -x "${SETUP_CAN}" ]]; then
            "${SETUP_CAN}" test --iface "${IFACE}" || true
        else
            warn "setup_can.sh not found or not executable; skipping CAN test"
        fi
    fi
}

uninstall_service() {
    require_root
    require_cmd systemctl

    log "+ systemctl disable --now $(service_name)"
    systemctl disable --now "$(service_name)" || true
    rm -f "$(service_file)" "$(env_file)"
    rmdir "${ENV_DIR}" 2>/dev/null || true
    log "+ systemctl daemon-reload"
    systemctl daemon-reload
}

status_service() {
    require_cmd systemctl
    systemctl status "$(service_name)" --no-pager || true
    if command -v ip >/dev/null 2>&1; then
        ip -details -statistics link show "${IFACE}" || true
    fi
}

restart_service() {
    require_root
    require_cmd systemctl
    log "+ systemctl restart $(service_name)"
    systemctl restart "$(service_name)"
    status_service
}

show_service() {
    local env_path
    local svc_path
    env_path="$(env_file)"
    svc_path="$(service_file)"

    if [[ -f "${env_path}" ]]; then
        printf '# %s\n' "${env_path}"
        cat "${env_path}"
    else
        warn "config file not found: ${env_path}"
    fi

    if [[ -f "${svc_path}" ]]; then
        printf '\n# %s\n' "${svc_path}"
        cat "${svc_path}"
    else
        warn "service file not found: ${svc_path}"
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
            --yes|-y)
                YES=1
                shift
                ;;
            --dry-run)
                DRY_RUN=1
                shift
                ;;
            --test)
                RUN_TEST=1
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

case "${COMMAND}" in
    install)
        install_service
        ;;
    uninstall)
        uninstall_service
        ;;
    status)
        status_service
        ;;
    restart)
        restart_service
        ;;
    show)
        show_service
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        die "unknown command '${COMMAND}'"
        ;;
esac
