#!/bin/bash
PERSISTENT_BASE="${HOME}/.elibotsim"
IP_ADDRESS="192.168.56.101"
PORT_FORWARDING_WITH_DASHBOARD="-p 30001-30004:30001-30004 -p 29999:29999 -p 6080:6080 -p 2222:22"
#PORT_FORWARDING_WITHOUT_DASHBOARD="-p 30001-30004:30001-30004"
CONTAINER_NAME="elibotsim"
SERIES_FLAG=0
ROBOT_MODEL=""
ROBOT_SERIES=""
IMAGE_VERSION="2.14.5"
PORT_FORWARDING=""
PROGRAM_STORAGE_ARG=""
ELITECO_STORAGE_ARG=""
DETACHED=false
TOOL_TYPE_ID=${ARG_TOOL_TYPE_ID:-1}
CONTROL_BOX_ID=${ARG_CONTROL_BOX_ID:-1}
ROBOT_GENERATION_ID="${SERIES_FLAG:-0}"
REGISTRY_IMAGE_BASE="eliterobots/cs-series-simulator"

help() {
  echo "Starts ELIBOTSim inside a docker container"
  echo
  echo "Syntax: $(basename "$0") [-m|s|h]"
  echo "options:"
  echo "  -m <model>   Robot model (CS63, CS66, ES66, etc). Defaults to CS66."
  echo "  -c           CONTROL_BOX_ID. Defaults to 1"
  echo "  -t           TOOL_TYPE_ID. Defaults to 1"
  echo "  -v <version> Image tag to use. Defaults to '2.14.5'"
  echo "  -p <folder>  Program dir on host"
  echo "  -u <folder>  ELITECO dir on host"
  echo "  -n           Container name. Default: '$CONTAINER_NAME'"
  echo "  -i           Container IP. Default: $IP_ADDRESS"
  echo "  -d           Detached mode (background)"
  echo "  -f           Port forwarding string (e.g. \"-p 6080:6080 -p 5900:5900\")"
  echo "  -r <0|1>     Robot series flag. 0 = CS_SERIES, 1 = ES_SERIES (auto overridden by -m)"
  echo "  -R <base>    Registry image base. Default: '$REGISTRY_IMAGE_BASE'"
  echo "  -h           Show this help"
}

parse_arguments() {
  while getopts ":hm:v:p:u:i:f:n:dt:c:r:" option; do
    case $option in
      h) help; exit ;;
      m) ROBOT_MODEL=${OPTARG} ;;
      v) IMAGE_VERSION=${OPTARG} ;;
      p) PROGRAM_STORAGE_ARG=${OPTARG} ;;
      u) ELITECO_STORAGE_ARG=${OPTARG} ;;
      i) IP_ADDRESS=${OPTARG} ;;
      f) PORT_FORWARDING=${OPTARG} ;;
      n) CONTAINER_NAME=${OPTARG} ;;
      d) DETACHED=true ;;
      t) TOOL_TYPE_ID=${OPTARG} ;;
      c) CONTROL_BOX_ID=${OPTARG} ;;
      r) SERIES_FLAG=${OPTARG} ;;
      R) REGISTRY_IMAGE_BASE=${OPTARG} ;;
      \?) echo "Error: Invalid option"; help; exit 1 ;;
    esac
  done
}

fill_information() {
  if [ -z "$ROBOT_MODEL" ]; then
    ROBOT_MODEL="CS66"
  fi
  ROBOT_MODEL=$(echo "$ROBOT_MODEL" | tr -d '[:space:]' | tr '[:lower:]' '[:upper:]')
  if [[ "$ROBOT_MODEL" =~ ^CS[0-9A-Z-]*$ ]]; then
    SERIES_FLAG=0
    ROBOT_SERIES="CS_SERIES"
  elif [[ "$ROBOT_MODEL" =~ ^ES[0-9A-Z-]*$ ]]; then
    SERIES_FLAG=1
    ROBOT_SERIES="ES_SERIES"
  else
    SERIES_FLAG=0
    ROBOT_SERIES="CS_SERIES"
  fi
  ROBOT_GENERATION_ID="$SERIES_FLAG"
  ROBOT_TYPE_ID=$(get_robot_type_id_from_robot_model "$ROBOT_MODEL") || {
    echo "Invalid robot model: '$ROBOT_MODEL'"
    exit 1
  }
}

validate_ip_address() {
  local ip="$1"
  if ! [[ "$ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Invalid IP format: $ip"
    exit 1
  fi
  IFS='.' read -r a b c d <<< "$ip"
  if [[ "$a" -ne 192 || "$b" -ne 168 || "$c" -ne 56 ]]; then
    echo "Invalid IP range: $ip"
    exit 1
  fi
  if (( d < 1 || d > 254 )); then
    echo "Invalid last octet: $d"
    exit 1
  fi
}

resolve_docker_image() {
  local version="$1"

  if docker image inspect "elite_sim:${version}" >/dev/null 2>&1; then
    echo "elite_sim:${version}"
    return
  fi
  

  if docker image inspect "${REGISTRY_IMAGE_BASE}:${version}" >/dev/null 2>&1; then
    echo "${REGISTRY_IMAGE_BASE}:${version}"
    return
  fi
  

  echo "Pulling ${REGISTRY_IMAGE_BASE}:${version}..." >&2
   if ! docker pull "${REGISTRY_IMAGE_BASE}:${version}" 1>&2; then
    echo "Error:Can not pull${REGISTRY_IMAGE_BASE}:${version}" >&2
    return 1
  fi

  docker tag "${REGISTRY_IMAGE_BASE}:${version}" "elite_sim:${version}" >/dev/null 2>&1
  docker rmi "${REGISTRY_IMAGE_BASE}:${version}" >/dev/null 2>&1 || true
  echo "elite_sim:${version}"
}
get_robot_type_id_from_robot_model() {
  local model="$1"
  model=$(echo "$model" | tr -d '[:space:]' | tr '[:lower:]' '[:upper:]')
  if [[ "$model" =~ ^ES ]]; then
    model="CS${model#ES}"
  fi
  case "$model" in
    CS63) echo 6203 ;;
    CS66) echo 6206 ;;
    CS68) echo 6208 ;;
    CS612) echo 6212 ;;
    CS616) echo 6216 ;;
    CS616-9) echo 62162 ;;
    CS618) echo 6218 ;;
    CS620) echo 6220 ;;
    CS625) echo 6225 ;;
    CS630) echo 6230 ;;
    CS66A) echo 6106 ;;
    CS610A) echo 6110 ;;
    CS612A) echo 6112 ;;
    CS66AZ) echo 61061 ;;
    CS612AZ) echo 61121 ;;
    CS56H) echo 5206 ;;
    CS520H) echo 5220 ;;
    CS525H) echo 5225 ;;
    CS530H) echo 5230 ;;
    *) echo "Unknown model: $model" >&2; return 1 ;;
  esac
}

main() {
  parse_arguments "$@"
  fill_information
  validate_ip_address "$IP_ADDRESS"

  ELITECO_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/eliteco"
  PROGRAM_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/${ROBOT_MODEL}/programs"
  [ -n "$PROGRAM_STORAGE_ARG" ] && PROGRAM_STORAGE="$PROGRAM_STORAGE_ARG"
  [ -n "$ELITECO_STORAGE_ARG" ] && ELITECO_STORAGE="$ELITECO_STORAGE_ARG"

  if [ -z "$PORT_FORWARDING" ]; then
    if [ "$ROBOT_SERIES" = "CS_SERIES" ]; then
      PORT_FORWARDING=$PORT_FORWARDING_WITH_DASHBOARD
    else
      PORT_FORWARDING=$PORT_FORWARDING_WITH_DASHBOARD
    fi
  fi

  if docker network inspect elibotsim_net &>/dev/null; then
    docker network rm elibotsim_net || exit 1
  fi

  docker network create --subnet=192.168.56.0/24 elibotsim_net &>/dev/null || {
     echo "Failed to create network"; 
     exit 1
            }
  mkdir -p "${ELITECO_STORAGE}" "${PROGRAM_STORAGE}"
  ELITECO_STORAGE=$(realpath "$ELITECO_STORAGE")
  PROGRAM_STORAGE=$(realpath "$PROGRAM_STORAGE")
  echo -e "IMAGE_VERSION: \e[35m$IMAGE_VERSION\e[0m"
  #echo -e "ROBOT_SERIES:  \e[35m$ROBOT_SERIES\e[0m"
  echo -e "ROBOT_MODEL:   \e[35m$ROBOT_MODEL\e[0m"
  echo -e "ROBOT_TYPE_ID: \e[35m$ROBOT_TYPE_ID\e[0m"
  echo -e "TOOL_TYPE_ID:  \e[35m$TOOL_TYPE_ID\e[0m"
  echo -e "CONTROL_BOX_ID:\e[35m$CONTROL_BOX_ID\e[0m"
  echo -e "ROBOT_GENERATION_ID: \e[35m$ROBOT_GENERATION_ID\e[0m"
  docker_args=(
    run --rm
    --net elibotsim_net --ip "$IP_ADDRESS"
    -v "${ELITECO_STORAGE}:/opt/EliteRobots/EliRobot/.plugins"
    -v "${PROGRAM_STORAGE}:/opt/EliteRobots/EliRobot/program"
    -e "ROBOT_TYPE_ID=${ROBOT_TYPE_ID}"
    -e "TOOL_TYPE_ID=${TOOL_TYPE_ID}"
    -e "CONTROL_BOX_ID=${CONTROL_BOX_ID}"
    -e "ROBOT_GENERATION_ID=${ROBOT_GENERATION_ID}"
    --name "$CONTAINER_NAME"
  )
  [ -n "$PORT_FORWARDING" ] && docker_args+=($PORT_FORWARDING)
  [ "$DETACHED" = true ] && docker_args+=(-d)

  REAL_IMAGE=$(resolve_docker_image "$IMAGE_VERSION") || { echo " Image resolve/pull failed, aborting."; exit 2; }
  docker_args+=("$REAL_IMAGE")

  if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Container ${CONTAINER_NAME} already exists. Removing it..."
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  fi

  docker_cmd="docker ${docker_args[*]}"

  echo -e "\e[32m✅ $CONTAINER_NAME container starting...\e[0m"
  echo -e "   Name: \e[32m$CONTAINER_NAME\e[0m"
  echo -e "   IP:   \e[32m$IP_ADDRESS\e[0m"
  echo -e "   Web :\e[32m http://$IP_ADDRESS:6080 or http://localhost:6080\e[0m"
  echo -e "   To exit, press CTRL+C"

  if $docker_cmd >/dev/null 2>&1; then
    echo "-----------end----------"
  else
    echo "❌ Failed to start container: $CONTAINER_NAME"
    exit 2
  fi


  TRAP_CMD="
  echo 'Stopping elibotsim ...';
  docker stop '$CONTAINER_NAME' >/dev/null 2>&1 || true
  docker kill '$CONTAINER_NAME' >/dev/null 2>&1 || true
  docker container wait '$CONTAINER_NAME' >/dev/null 2>&1 || true
  echo 'done';
  exit
  "
  trap "$TRAP_CMD" SIGINT SIGTERM

  if [ "$DETACHED" = false ]; then
    echo "To exit, press CTRL+C"
    while :; do sleep 1; done
  else
    echo "To kill it, please execute 'docker stop $CONTAINER_NAME'"
  fi
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  main "$@"
fi


